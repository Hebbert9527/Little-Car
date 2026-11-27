#include "w4_maze.h"
#include "w4_rtos.h"
#include "w4_integration.h"
#include "planner_config.h"
extern char test_map[192];
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
extern bool test_imu_valid;

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
volatile unsigned maze_test_tick,maze_test_stage;
typedef struct { unsigned ticks; float x,y,clearance; unsigned state,fault,replans; } TestOutcome;
volatile TestOutcome maze_outcomes[6];

static void drive_room(bool with_board) {
    for(unsigned i=0;i<4;++i) room_scan(with_board);
    send(with_board?"MAZE,0,1.05\n":"MAZE,0,0.65\n");
    W4Path_Poll(); W4Path_Poll();
    assert(strstr(test_ack,"MAZE_ACCEPTED"));
    float counts_l=test_left_count,counts_r=test_right_count,vl=0,vr=0,min_clearance=10;
    bool went_around=false;
    for(unsigned tick=0;tick<4000 && w4_nav.state!=W4_ARRIVED;++tick) {
        maze_test_tick=tick;
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
            min_clearance=fminf(min_clearance,hypotf(side,ahead));
            assert(hypotf(side,ahead)>=0.17f);
            if(fabsf(ahead)<0.10f && fabsf(w4_nav.pose.x_m)>=0.37f)
                went_around=true;
        }
        W4Path_Poll();
        assert(w4_nav.fault!=1); assert(w4_nav.fault!=5); assert(w4_nav.fault!=9); assert(w4_nav.fault!=10); assert(w4_nav.fault!=11); assert(w4_nav.state!=W4_FAULT);
    }
    float goal_y=with_board?1.05f:0.65f;
    assert(w4_nav.state==W4_ARRIVED && hypotf(w4_nav.pose.x_m,w4_nav.pose.y_m-goal_y)<0.08f);
    if(with_board) assert(went_around);
    assert(test_pwm_l==0 && test_pwm_r==0);
    maze_outcomes[with_board?1:0]=(TestOutcome){maze_test_tick,w4_nav.pose.x_m,w4_nav.pose.y_m,with_board?min_clearance:0,w4_nav.state,w4_nav.fault,0};
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
static void test_route_contract(void) {
    reset_run(); scan(false); scan(false);
    send("MAZE,0,0.8\n"); W4Path_Poll(); W4Path_Poll();
    assert(strstr(test_ack,"MAZE_ACCEPTED"));
    assert(W4Maze_RouteCount()>2);
    assert(W4Maze_FinalLeg());
    W4Waypoint unused;
    assert(osMessageQueueGet(Q_Path,&unused,0,0)!=osOK);
    unsigned id=w4_nav.cmd_id;
    test_ms+=10; W4Control_Step(test_ms);
    send("GOTO,1,1\n");
    assert(strstr(test_ack,"BUSY,NAV_ACTIVE") && w4_nav.cmd_id==id);
    /* Crossing intermediate points must keep the full route active. */
    for(unsigned i=1;i<=6;++i) {
        w4_nav.pose.y_m=0.1f*i;
        test_ms+=10; W4Control_Step(test_ms);
        assert(w4_nav.state==W4_TRACKING);
    }
    send("STOP\n"); W4Control_Step(test_ms); W4Path_Poll();
    assert(w4_nav.state==W4_STOPPED && test_pwm_l==0 && test_pwm_r==0);
}
static void test_goal_xy_only(void) {
    reset_run(); scan(false); scan(false);
    send("MAZE,0,0.3\n"); W4Path_Poll(); W4Path_Poll();
    test_ms+=10; W4Control_Step(test_ms);
    w4_nav.pose.y_m=0.29f;
    test_heading=0.25f;
    for(unsigned tick=0;tick<23;++tick) {
        test_ms+=10; W4Control_Step(test_ms);
        assert(test_pwm_l==0 && test_pwm_r==0);
    }
    assert(w4_nav.state==W4_ARRIVED);
}
/* Ray-cast two poles or alternating maze walls; integrate unequal loaded
   wheel responses using the unchanged production motor controller. */
typedef struct { float ax,ay,bx,by; } TestWall;
static const TestWall maze_walls[]={{-1.2f,0.50f,0.10f,0.50f},{-0.10f,1.10f,1.2f,1.10f}};
static float wall_clearance(float x,float y,TestWall b) {
    float vx=b.bx-b.ax,vy=b.by-b.ay;
    float t=fmaxf(0,fminf(1,((x-b.ax)*vx+(y-b.ay)*vy)/(vx*vx+vy*vy)));
    return hypotf(x-b.ax-t*vx,y-b.ay-t*vy);
}
static void course_scan(unsigned course,bool obstacles) {
    float x=w4_nav.pose.x_m,y=w4_nav.pose.y_m;
    for(int deg=0;deg<360;deg+=3) {
        float angle=w4_nav.pose.theta_rad-deg*W4_PI/180;
        float c=cosf(angle),d=sinf(angle);
        float distance=fminf((copysignf(1.8f,c)-x)/c,(copysignf(2.1f,d)-y)/d);
        if(obstacles) for(unsigned i=0;i<2;++i) {
            if(course==1) {
                float cx=i?0.06f:-0.06f,cy=i?1.10f:0.50f;
                float vx=cx-x,vy=cy-y,along=vx*c+vy*d;
                float disc=0.06f*0.06f-(vx*vx+vy*vy-along*along);
                if(disc>=0 && along>0) distance=fminf(distance,along-sqrtf(disc));
            } else {
                TestWall b=maze_walls[i];
                if(fabsf(d)>0.0001f) {
                    float t=(b.ay-y)/d,hx=x+t*c;
                    if(t>0 && hx>=b.ax && hx<=b.bx) distance=fminf(distance,t);
                }
            }
        }
        W4Maze_Point((float)deg,1000*distance,12,deg==0,test_ms);
    }
    W4Path_Poll();
}
static void drive_course(unsigned course,bool dynamic) {
    reset_run();
    for(unsigned i=0;i<4;++i) course_scan(course,!dynamic);
    send("MAZE,0,1.65\n"); W4Path_Poll(); W4Path_Poll();
    assert(strstr(test_ack,"MAZE_ACCEPTED"));
    float counts_l=test_left_count,counts_r=test_right_count,vl=0,vr=0,min_distance=10;
    unsigned replans=0,previous_state=W4_PLANNING;
    for(unsigned tick=0;tick<11000 && w4_nav.state!=W4_ARRIVED;++tick) {
        maze_test_tick=tick;
        vl+=0.08f*(copysignf(1.65f*fmaxf(0,fabsf(test_pwm_l)-0.08f),test_pwm_l)-vl);
        vr+=0.08f*(copysignf(1.34f*fmaxf(0,fabsf(test_pwm_r)-0.08f),test_pwm_r)-vr);
        counts_l+=vl*.01f*w4_config.cpr_r/(W4_PI*w4_config.diameter_r_m);
        counts_r+=vr*.01f*w4_config.cpr_l/(W4_PI*w4_config.diameter_l_m);
        test_left_count=(uint16_t)lroundf(counts_l); test_right_count=(uint16_t)lroundf(counts_r);
        test_gyro=(vr-vl)/w4_config.track_m; test_heading=W4Wrap(test_heading+.01f*test_gyro);
        test_ms+=10; W4Control_Step(test_ms);
        if(tick%20==0) course_scan(course,!dynamic||tick>=20);
        if(!dynamic||tick>=20) for(unsigned i=0;i<2;++i) {
            float distance=course==1?
                hypotf(w4_nav.pose.x_m-(i?0.06f:-0.06f),w4_nav.pose.y_m-(i?1.10f:0.50f))-0.06f:
                wall_clearance(w4_nav.pose.x_m,w4_nav.pose.y_m,maze_walls[i]);
            min_distance=fminf(distance,min_distance);
            assert(distance>=0.17f);
        }
        W4Path_Poll();
        if(previous_state==W4_TRACKING && w4_nav.state==W4_PLANNING) ++replans;
        previous_state=w4_nav.state;
        assert(w4_nav.fault!=5); assert(w4_nav.fault!=9); assert(w4_nav.fault!=10); assert(w4_nav.fault!=11);
        assert(w4_nav.state!=W4_FAULT);
    }
    assert(w4_nav.state==W4_ARRIVED);
    assert(hypotf(w4_nav.pose.x_m,w4_nav.pose.y_m-1.65f)<0.08f);
    assert(test_pwm_l==0 && test_pwm_r==0);
    if(dynamic) assert(replans>0);
    maze_outcomes[dynamic?4:course+1]=(TestOutcome){maze_test_tick,w4_nav.pose.x_m,w4_nav.pose.y_m,min_distance,w4_nav.state,w4_nav.fault,replans};
}
static void test_planner_edges(void) {
    reset_run(); scan(false); scan(false);
    /* Same-cell endpoints must retain the exact goal, not replace it with start. */
    send("MAZE,0.01,0.04\n"); W4Path_Poll(); W4Path_Poll();
    assert(strstr(test_ack,"MAZE_ACCEPTED"));
    test_ms+=10; W4Control_Step(test_ms);
    for(unsigned i=0;i<22;++i) { test_ms+=10; W4Control_Step(test_ms); }
    assert(w4_nav.state==W4_ARRIVED);
    reset_run();
    for(unsigned i=0;i<4;++i) room_scan(true);
    send("MAZE,0,0.55\n"); W4Path_Poll(); W4Path_Poll();
    assert(w4_nav.state==W4_FAULT && w4_nav.fault==10);
    assert(test_pwm_l==0 && test_pwm_r==0);
    reset_run(); scan(false); scan(false);
    send("MAZE,0,0.8\n"); W4Path_Poll(); W4Path_Poll();
    test_ms+=10; W4Control_Step(test_ms);
    test_imu_valid=false;
    test_ms+=10; W4Control_Step(test_ms);
    assert(w4_nav.state==W4_FAULT && w4_nav.fault==1);
    assert(test_pwm_l==0 && test_pwm_r==0);
    test_imu_valid=true;
}


static char mapped_cell(unsigned wanted_row,unsigned wanted_col) {
    for(unsigned i=0;i<2*MAX_GRID_SIZE+1;++i) {
        test_ms+=100; W4Control_Step(test_ms); W4Path_Poll();
        unsigned long version,age; unsigned row,res,center;
        long dx,dy; char cells[MAX_GRID_SIZE+1];
        int n=sscanf(test_map,"M,%lu,%u,%ld,%ld,%u,%u,%lu,%107s",
                     &version,&row,&dx,&dy,&res,&center,&age,cells);
        assert(n==8 && res==100 && center==MAX_GRID_SIZE/2);
        if(row==wanted_row) return cells[wanted_col];
    }
    assert(0); return '?';
}
static void test_persistent_walls(void) {
    reset_run();
    for(unsigned i=0;i<5;++i) scan(true);
    assert(mapped_cell(MAX_GRID_SIZE/2+3,MAX_GRID_SIZE/2)=='#');
    // Many clear rays used to decrement this confirmed wall back to free.
    for(unsigned i=0;i<20;++i) scan(false);
    assert(mapped_cell(MAX_GRID_SIZE/2+3,MAX_GRID_SIZE/2)=='#');
    assert(!W4Maze_Fresh(test_ms)); // Cached telemetry must not refresh scans.
    test_imu_valid=false; test_ms+=10; W4Control_Step(test_ms);
    assert(mapped_cell(MAX_GRID_SIZE/2+3,MAX_GRID_SIZE/2)=='#');
    assert(test_pwm_l==0 && test_pwm_r==0);
    test_imu_valid=true; reset_run();
    assert(mapped_cell(MAX_GRID_SIZE/2+3,MAX_GRID_SIZE/2)=='?');
}
static void open_scan(void) {
    for(int angle=0;angle<360;angle+=3)
        W4Maze_Point((float)angle,6500,12,angle==0,test_ms);
    W4Path_Poll();
}
static void test_five_metre_targets(void) {
    const char *goals[]={"MAZE,0,5\n","MAZE,5,0\n","MAZE,-5,0\n","MAZE,0,-5\n",
                         "MAZE,5,5\n","MAZE,-5,5\n","MAZE,5,-5\n","MAZE,-5,-5\n"};
    W4Command c;
    assert(W4ParseCommand("MAZE,5,-5",&c) && c.x_m==5 && c.y_m==-5);
    assert(!W4ParseCommand("MAZE,5.01,0",&c));
    assert(!W4ParseCommand("MAZE,0,-5.01",&c));
    for(unsigned i=0;i<sizeof goals/sizeof goals[0];++i) {
        reset_run(); open_scan(); open_scan();
        send(goals[i]); W4Path_Poll(); W4Path_Poll();
        assert(strstr(test_ack,"MAZE_ACCEPTED"));
        assert(W4Maze_RouteCount()>=2);
        assert(W4Maze_LoadRoute(w4_nav.cmd_id,&w4_nav.pose));
    }
    reset_run();
}

int main(void) {
    test_ms=100;
    W4Calibration_Apply(); W4Nav_Init(); assert(W4HardwareReady());
    send("MAZE,0,0.8\n"); W4Path_Poll(); W4Path_Poll();
    assert(w4_nav.state==W4_FAULT && w4_nav.fault==9);
#ifdef TEST_DYNAMIC_ONLY
    maze_test_stage=7; drive_course(1,true);
    return 0;
#endif
    maze_test_stage=1; test_route_contract();
    maze_test_stage=2; reset_run(); drive_room(false);
    maze_test_stage=3; reset_run(); drive_room(true);
    maze_test_stage=4; test_goal_xy_only();
    maze_test_stage=5; drive_course(1,false);
    maze_test_stage=6; drive_course(2,false);
    maze_test_stage=7; drive_course(1,true);
    maze_test_stage=8; test_planner_edges();
    maze_test_stage=9; test_persistent_walls();
    maze_test_stage=10; test_five_metre_targets();
    reset_run(); assert(!W4Maze_Fresh(test_ms));
    return 0;
}
