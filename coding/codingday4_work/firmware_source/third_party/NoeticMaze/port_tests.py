from pathlib import Path
R=Path(r"D:\EMBEGGed\week4codingday\coding\coding(1)\coding\codingday4_work\firmware_source")
old=(R/"build/noetic_port/before/test_maze.c").read_text()
s=old[:old.index("static W4Waypoint start_open_leg")]
s=s.replace("static void drive_room(bool with_board) {","volatile unsigned maze_test_tick,maze_test_stage;\nstatic void drive_room(bool with_board) {")
s=s.replace("        vl+=0.08f","        maze_test_tick=tick;\n        vl+=0.08f")
s=s.replace("assert(w4_nav.state!=W4_FAULT);","assert(w4_nav.fault!=1); assert(w4_nav.fault!=5); assert(w4_nav.fault!=9); assert(w4_nav.fault!=10); assert(w4_nav.fault!=11); assert(w4_nav.state!=W4_FAULT);")
s+='''static void test_route_contract(void) {
    reset_run(); scan(false); scan(false);
    send("MAZE,0,0.8\\n"); W4Path_Poll(); W4Path_Poll();
    assert(strstr(test_ack,"MAZE_ACCEPTED"));
    assert(W4Maze_RouteCount()>2);
    assert(W4Maze_FinalLeg());
    W4Waypoint unused;
    assert(osMessageQueueGet(Q_Path,&unused,0,0)!=osOK);
    unsigned id=w4_nav.cmd_id;
    test_ms+=10; W4Control_Step(test_ms);
    send("GOTO,1,1\\n");
    assert(strstr(test_ack,"BUSY,NAV_ACTIVE") && w4_nav.cmd_id==id);
    /* Crossing intermediate points must keep the full route active. */
    for(unsigned i=1;i<=6;++i) {
        w4_nav.pose.y_m=0.1f*i;
        test_ms+=10; W4Control_Step(test_ms);
        assert(w4_nav.state==W4_TRACKING);
    }
    send("STOP\\n"); W4Control_Step(test_ms); W4Path_Poll();
    assert(w4_nav.state==W4_STOPPED && test_pwm_l==0 && test_pwm_r==0);
}
static void test_goal_xy_only(void) {
    reset_run(); scan(false); scan(false);
    send("MAZE,0,0.3\\n"); W4Path_Poll(); W4Path_Poll();
    test_ms+=10; W4Control_Step(test_ms);
    w4_nav.pose.y_m=0.29f;
    test_heading=0.25f;
    for(unsigned tick=0;tick<23;++tick) {
        test_ms+=10; W4Control_Step(test_ms);
        assert(test_pwm_l==0 && test_pwm_r==0);
    }
    assert(w4_nav.state==W4_ARRIVED);
}
int main(void) {
    test_ms=100;
    W4Calibration_Apply(); W4Nav_Init(); assert(W4HardwareReady());
    send("MAZE,0,0.8\\n"); W4Path_Poll(); W4Path_Poll();
    assert(w4_nav.state==W4_FAULT && w4_nav.fault==9);
    maze_test_stage=1; test_route_contract();
    maze_test_stage=2; reset_run(); drive_room(false);
    maze_test_stage=3; reset_run(); drive_room(true);
    maze_test_stage=4; test_goal_xy_only();
    reset_run(); assert(!W4Maze_Fresh(test_ms));
    return 0;
}
'''
(R/"tests/navigation/test_maze.c").write_text(s,encoding="utf-8",newline="\n")
p=R.parent/"run_cortex_tests.py"
s=p.read_text()
s=s.replace('    if result:\n','''    if result:
        import json
        debug={}
        for name,fmt,size in [("w4_nav","<IIffIIfffffI",48),("maze_test_tick","<I",4),("maze_test_stage","<I",4),
                              ("test_pwm_l","<f",4),("test_pwm_r","<f",4)]:
            if name in symbols: debug[name]=struct.unpack(fmt,uc.mem_read(symbols[name],size))
        print(json.dumps(debug),flush=True)
''')
p.write_text(s,encoding="utf-8",newline="\n")
