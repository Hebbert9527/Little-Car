from pathlib import Path
R=Path(r"D:\EMBEGGed\week4codingday\coding\coding(1)\coding\codingday4_work\firmware_source")
p=R/"tests/navigation/test_maze.c"
s=p.read_text()
s=s.replace('volatile unsigned maze_test_tick,maze_test_stage;','''volatile unsigned maze_test_tick,maze_test_stage;
typedef struct { unsigned ticks; float x,y,clearance; unsigned state,fault,replans; } TestOutcome;
volatile TestOutcome maze_outcomes[6];
volatile unsigned maze_replans;
static unsigned last_route_length;''')
# no unused vars
s=s.replace("volatile unsigned maze_replans;\nstatic unsigned last_route_length;","")
s=s.replace('    if(with_board) return;','''    maze_outcomes[with_board?1:0]=(TestOutcome){maze_test_tick,w4_nav.pose.x_m,w4_nav.pose.y_m,with_board?0.17f:0,w4_nav.state,w4_nav.fault,0};
    if(with_board) return;''')
more='''/* Ray-cast two poles or alternating maze walls; integrate unequal loaded
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
    send("MAZE,0,1.65\\n"); W4Path_Poll(); W4Path_Poll();
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
    send("MAZE,0.01,0.04\\n"); W4Path_Poll(); W4Path_Poll();
    assert(strstr(test_ack,"MAZE_ACCEPTED"));
    test_ms+=10; W4Control_Step(test_ms);
    for(unsigned i=0;i<22;++i) { test_ms+=10; W4Control_Step(test_ms); }
    assert(w4_nav.state==W4_ARRIVED);
    reset_run();
    for(unsigned i=0;i<4;++i) room_scan(true);
    send("MAZE,0,0.55\\n"); W4Path_Poll(); W4Path_Poll();
    assert(w4_nav.state==W4_FAULT && w4_nav.fault==10);
    assert(test_pwm_l==0 && test_pwm_r==0);
}
'''
s=s.replace("int main(void) {",more+"\nint main(void) {")
s=s.replace("    reset_run(); assert(!W4Maze_Fresh(test_ms));","""    maze_test_stage=5; drive_course(1,false);
    maze_test_stage=6; drive_course(2,false);
    maze_test_stage=7; drive_course(1,true);
    maze_test_stage=8; test_planner_edges();
    reset_run(); assert(!W4Maze_Fresh(test_ms));""")
p.write_text(s,encoding="utf-8",newline="\n")
p=R.parent/"run_cortex_tests.py"
s=p.read_text().replace("4_000_000_000 if name==","12_000_000_000 if name==")
a='    result=struct.unpack("<I",uc.mem_read(symbols["test_result"],4))[0]'
s=s.replace(a,a+'''
    if "maze_outcomes" in symbols:
        import json
        rows=[]
        for i,name in enumerate(["open","board","poles","maze","dynamic","unused"]):
            row=struct.unpack("<IfffIII",uc.mem_read(symbols["maze_outcomes"]+28*i,28))
            rows.append(dict(zip(["ticks","x","y","min_clearance","state","fault","replans"],row),scenario=name))
        path.with_suffix(".outcomes.json").write_text(json.dumps(rows,indent=2))
''')
p.write_text(s,encoding="utf-8",newline="\n")
