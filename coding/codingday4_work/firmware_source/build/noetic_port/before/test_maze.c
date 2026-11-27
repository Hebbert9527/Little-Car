#include "w4_maze.h"
#include "w4_rtos.h"
#include "w4_integration.h"
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
extern uint32_t test_ms;
extern char test_ack[96];
extern uint16_t test_left_count,test_right_count;
extern float test_heading,test_gyro;
extern float test_pwm_l,test_pwm_r;

static void send(const char *s) {
    while(*s) W4Bt_OnByteISR((uint8_t)*s++);
    W4Bt_Process();
}
static void scan(bool obstacle) {
    for(int deg=0;deg<360;deg+=3) {
        float rad=deg*W4_PI/180.0f;
        float d=1800.0f;
        /* Inject the weak return before a valid point in the same bin.
           A stronger return at the same distance must still map as a wall. */
        if(deg==0 || deg==180)
            W4Maze_Point((float)deg,330.0f,2,deg==0,test_ms);
        if(obstacle && cosf(rad)>0.0f && fabsf(tanf(rad))<0.67f)
            d=330.0f/cosf(rad);
        W4Maze_Point((float)deg,d,12,false,test_ms);
    }
    W4Path_Poll();
}
/* Four static walls in a square room, sampled using the current odometry.
   Encoder and gyro feedback below come from the physical-channel PWM outputs. */
static void room_scan(bool with_board) {
    for(int deg=0;deg<360;deg+=6) {
        float a=w4_nav.pose.theta_rad-deg*W4_PI/180.0f;
        float c=cosf(a),s=sinf(a);
        float dx=(copysignf(1.4f,c)-w4_nav.pose.x_m)/c;
        float dy=(copysignf(1.4f,s)-w4_nav.pose.y_m)/s;
        float distance=fminf(dx,dy);
        if(with_board && fabsf(s)>0.001f) {
            float board_distance=(0.55f-w4_nav.pose.y_m)/s;
            float contact_x=w4_nav.pose.x_m+board_distance*c;
            if(board_distance>0 && fabsf(contact_x)<=0.20f)
                distance=fminf(distance,board_distance);
        }
        W4Maze_Point((float)deg,1000*distance,12,deg==0,test_ms);
    }
    W4Path_Poll();
}
static void drive_room(bool with_board) {
    for(unsigned i=0;i<4;++i) room_scan(with_board);
    send(with_board?"MAZE,0,1.05\n":"MAZE,0,0.65\n");
    W4Path_Poll(); W4Path_Poll();
    assert(strstr(test_ack,"MAZE_ACCEPTED"));
    float counts_l=test_left_count,counts_r=test_right_count,vl=0,vr=0;
    bool went_around=false;
    for(unsigned tick=0;tick<4000 && w4_nav.state!=W4_ARRIVED;++tick) {
        vl+=0.08f*(copysignf(1.65f*fmaxf(0,fabsf(test_pwm_l)-0.08f),test_pwm_l)-vl);
        vr+=0.08f*(copysignf(1.34f*fmaxf(0,fabsf(test_pwm_r)-0.08f),test_pwm_r)-vr);
        counts_l+=vl*.01f*w4_config.cpr_r/(W4_PI*w4_config.diameter_r_m);
        counts_r+=vr*.01f*w4_config.cpr_l/(W4_PI*w4_config.diameter_l_m);
        test_left_count=(uint16_t)lroundf(counts_l); test_right_count=(uint16_t)lroundf(counts_r);
        test_gyro=(vr-vl)/w4_config.track_m; test_heading=W4Wrap(test_heading+.01f*test_gyro);
        test_ms+=10;
        W4Control_Step(test_ms);
        if(tick%20==0) room_scan(with_board);
        if(with_board) {
            float side=fmaxf(0,fabsf(w4_nav.pose.x_m)-0.20f);
            float ahead=w4_nav.pose.y_m-0.55f;
            assert(hypotf(side,ahead)>=0.17f);
            if(fabsf(ahead)<0.10f && fabsf(w4_nav.pose.x_m)>=0.37f)
                went_around=true;
        }
        W4Path_Poll();
        assert(w4_nav.state!=W4_FAULT);
    }
    float goal_y=with_board?1.05f:0.65f;
    assert(w4_nav.state==W4_ARRIVED && hypotf(w4_nav.pose.x_m,w4_nav.pose.y_m-goal_y)<0.08f);
    if(with_board) assert(went_around);
    assert(test_pwm_l==0 && test_pwm_r==0);
    if(with_board) return;
    send("MAZE,0,0.9\n"); W4Path_Poll(); W4Path_Poll();
    for(unsigned t=0;t<50;++t) { test_ms+=10; W4Control_Step(test_ms); W4Path_Poll(); }
    assert(w4_nav.state==W4_FAULT && w4_nav.fault==9 && test_pwm_l==0 && test_pwm_r==0);
}

static void reset_run(void) {
    send("STOP\n"); W4Control_Step(test_ms); W4Path_Poll();
    test_gyro=0;
    send("RESET\n"); W4Control_Step(test_ms); W4Path_Poll();
}
static W4Waypoint start_open_leg(const char *command) {
    reset_run();
    scan(false); scan(false);
    send(command); W4Path_Poll(); W4Path_Poll();
    assert(strstr(test_ack,"MAZE_ACCEPTED"));
    W4Waypoint next;
    assert(osMessageQueueGet(Q_Path,&next,0,0)==osOK);
    assert(osMessageQueuePut(Q_Path,&next,0,0)==osOK);
    test_ms+=10; W4Control_Step(test_ms);
    return next;
}
static void place(float x,float y,float heading) {
    w4_nav.pose.x_m=x; w4_nav.pose.y_m=y; w4_nav.pose.theta_rad=heading;
    test_heading=W4Wrap(heading-w4_config.imu_offset_rad);
}
static void test_maze_handoff(void) {
    /* Both intermediate and final XY goals accept a stopped chassis with a
       heading offset. They must never drive a pivot to restore the old line. */
    for(unsigned final=0;final<2;++final) {
        W4Waypoint next=start_open_leg(final?"MAZE,0,0.3\n":"MAZE,0,0.8\n");
        assert(W4Maze_FinalLeg()==(final!=0));
        place(next.x_m,next.y_m-0.01f,W4_PI/2+0.25f);
        for(unsigned tick=0;tick<22;++tick) {
            test_ms+=10; W4Control_Step(test_ms);
            assert(test_pwm_l==0 && test_pwm_r==0);
        }
        assert(w4_nav.state==(final?W4_ARRIVED:W4_PLANNING));
    }
}
static void test_maze_turn_threshold(void) {
    (void)start_open_leg("MAZE,0,0.8\n");
    uint32_t cmd=w4_nav.cmd_id;
    send("GOTO,1,1\n");
    assert(strstr(test_ack,"BUSY,NAV_ACTIVE"));
    assert(w4_nav.cmd_id==cmd && W4Maze_Active(cmd));
    long previous_mean=0;
    for(unsigned tick=0;tick<30;++tick) {
        /* Fluctuation around the old 0.50 rad pivot threshold. */
        float error=tick%2?0.52f:0.48f;
        place(0,0.1f,W4_PI/2-error);
        test_ms+=10; W4Control_Step(test_ms);
        char line[160]; unsigned long ms,id; long left,right;
        assert(W4Motor_FormatDiagnostics(line,sizeof line));
        assert(sscanf(line,"D,%lu,WHEEL,%lu,%ld,%ld",&ms,&id,&left,&right)==4);
        assert(left>0 && right>0);
        long mean=(left+right)/2;
        if(tick>0) assert(labs(mean-previous_mean)<=7);
        previous_mean=mean;
        if(tick%10==0) scan(false);
    }
    /* The former 0.85 rad threshold must not stop the moving chassis. */
    for(unsigned tick=0;tick<30;++tick) {
        float error=tick%2?0.90f:0.80f;
        place(0,0.1f,W4_PI/2-error);
        test_ms+=10; W4Control_Step(test_ms);
        char line[160]; unsigned long ms,id; long left,right;
        assert(W4Motor_FormatDiagnostics(line,sizeof line));
        assert(sscanf(line,"D,%lu,WHEEL,%lu,%ld,%ld",&ms,&id,&left,&right)==4);
        assert(left>0 && right>0);
        assert((left+right)/2>=64);
        if(tick%10==0) scan(false);
    }
    /* Ground commands 10/15 stopped 26-30 mm before the waypoint with 0.70 rad
       heading error. Both old requests (2/29 mm/s) were below useful drive. */
    for(unsigned tick=0;tick<40;++tick) {
        float error=tick<20?0.70f:-0.70f;
        place(0,tick<20?0.374f:0.370f,W4_PI/2-error);
        test_ms+=10; W4Control_Step(test_ms);
        char line[160]; unsigned long ms,id; long left,right;
        assert(W4Motor_FormatDiagnostics(line,sizeof line));
        assert(sscanf(line,"D,%lu,WHEEL,%lu,%ld,%ld",&ms,&id,&left,&right)==4);
        assert((left+right)/2>=64);
        assert(w4_nav.state==W4_TRACKING);
        if(tick%10==0) scan(false);
    }
    send("STOP\n"); test_ms+=10; W4Control_Step(test_ms); W4Path_Poll();
    assert(w4_nav.state==W4_STOPPED && test_pwm_l==0 && test_pwm_r==0);
}

int main(void) {
    test_ms=100;
    W4Calibration_Apply(); W4Nav_Init();
    assert(W4HardwareReady());
    send("MAZE,0,0.8\n"); W4Path_Poll(); W4Path_Poll();
    assert(w4_nav.state==W4_FAULT && w4_nav.fault==9);
    scan(false); scan(false);
    assert(W4Maze_Fresh(test_ms));
    send("MAZE,0,0.8\n"); W4Path_Poll(); W4Path_Poll();
    assert(strstr(test_ack,"MAZE_ACCEPTED"));
    W4Waypoint wp;
    assert(osMessageQueueGet(Q_Path,&wp,0,0)==osOK);
    assert(wp.y_m>0.25f && wp.y_m<0.47f && fabsf(wp.x_m)<0.01f);
    assert(!W4Maze_FinalLeg());
    w4_nav.state=W4_TRACKING;
    scan(true); scan(true);
    assert(W4Maze_LegBlocked());
    w4_nav.state=W4_PLANNING;
    W4Maze_NextLeg(); W4Path_Poll();
    assert(osMessageQueueGet(Q_Path,&wp,0,0)==osOK);
    assert(fabsf(wp.x_m)>0.1f); /* detour */
    assert(!W4Maze_LegBlocked());
    /* A stopped chassis at successive segment ends must reach the absolute
       goal via free cells. STOP always bypasses the planner. */
    for(int step=0;!W4Maze_FinalLeg() && step<20;++step) {
        w4_nav.pose.x_m=wp.x_m; w4_nav.pose.y_m=wp.y_m;
        W4Maze_NextLeg(); W4Path_Poll();
        assert(osMessageQueueGet(Q_Path,&wp,0,0)==osOK);
    }
    assert(W4Maze_FinalLeg() && fabsf(wp.y_m-0.8f)<0.01f);
    send("STOP\n"); W4Control_Step(test_ms); W4Path_Poll();
    assert(w4_nav.state==W4_STOPPED && test_pwm_l==0 && test_pwm_r==0);
    assert(!W4Maze_Fresh(test_ms+451));
    send("RESET\n"); W4Control_Step(test_ms); W4Path_Poll();
    assert(!W4Maze_Fresh(test_ms));
    drive_room(false);
    test_gyro=0;
    send("RESET\n"); W4Control_Step(test_ms); W4Path_Poll();
    drive_room(true);
    test_maze_handoff();
    test_maze_turn_threshold();
    return 0;
}
