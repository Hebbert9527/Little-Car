#include "w4_maze.h"
#include "w4_rtos.h"
#include "w4_integration.h"
#include <assert.h>
#include <math.h>
#include <string.h>
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
    return 0;
}
