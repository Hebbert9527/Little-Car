from pathlib import Path
import re
R=Path(r"D:\EMBEGGed\week4codingday\coding\coding(1)\coding\codingday4_work\firmware_source")
def write(p,s): (R/p).write_text(s,encoding="utf-8",newline="\n")
s=(R/"build/noetic_port/before/w4_maze.c").read_text()
s=s.replace('#include "w4_rtos.h"','#include "w4_rtos.h"\n#include "w4_noetic_motion.h"\n#include "planner_algo.h"\n#include "path_smoother.h"')
s=s.replace("static int16_t first_leg[CELLS],cost[CELLS];\nstatic uint8_t closed[CELLS],wall[CELLS];",'''static uint8_t wall[CELLS];
PlannerMemoryPool g_mem_pool;
static uint8_t planner_grid[MAX_MAP_BYTES];
static ServerMap planner_map={planner_grid,CELL_M,WIDTH};
static Point2D raw_path[MAX_PATH_LEN],previous_path[MAX_PATH_LEN],route[MAX_PATH_LEN];
static unsigned previous_len,route_len;
static volatile bool route_ready;
static uint32_t plan_stamp;
static volatile unsigned route_index;
static Point2D track_aim;
static bool aim_valid;
static float route_offset_x,route_offset_y;
static unsigned route_number;
''')
s=s.replace("void W4Maze_NextLeg(void) { blocked=false; need_leg=true; }","void W4Maze_NextLeg(void) { route_ready=false; need_leg=true; }")
s=s.replace("void W4Maze_Begin(uint32_t id) { active=id; ack_pending=true; W4Maze_NextLeg(); }","void W4Maze_Begin(uint32_t id) { active=id; ack_pending=true; blocked=false; previous_len=0; route_number=0; W4Maze_NextLeg(); }")
s=s.replace("void W4Maze_Reset(void) { ++epoch; active=0; fresh=false; }","void W4Maze_Reset(void) { ++epoch; active=0; fresh=false; route_ready=false; aim_valid=false; }")
a=s.index("static int manhattan")
b=s.index("void W4Maze_Poll",a)
s=s[:a]+'''/* NoeticMaze uses positive, x-major planner coordinates; Week4 retains
   its centered y-major telemetry map and +Y initial heading. */
static const float planner_origin=(CENTER+0.5f)*CELL_M;
static bool collision_line(Point2D a,Point2D b) {
    return bresenham_line_check(&planner_map,a,b,PHYSICAL_RADIUS,0.0f,false);
}
static bool route_safe(const Point2D *p,unsigned count) {
    if(count<2 || count>MAX_PATH_LEN) return false;
    for(unsigned i=1;i<count;++i)
        if(!isfinite(p[i].x)||!isfinite(p[i].y)||collision_line(p[i-1],p[i])) return false;
    return true;
}
static bool plan(float x,float y,float gx,float gy) {
    memset(planner_grid,0,sizeof planner_grid);
    for(int row=0;row<WIDTH;++row) for(int col=0;col<WIDTH;++col) {
        int i=cell(col,row);
        set_grid_val(&planner_map,col,row,(grid[i]>0||hit_scans[i])?1:grid[i]<0?2:0);
    }
    Point2D start={x+planner_origin,y+planner_origin};
    Point2D goal={gx+planner_origin,gy+planner_origin};
    int count=astar_plan(&planner_map,start,goal,previous_path,(int)previous_len,false,raw_path);
    if(count<2 || !route_safe(raw_path,(unsigned)count)) return false;
    memcpy(previous_path,raw_path,(size_t)count*sizeof(Point2D));
    previous_len=(unsigned)count;
    int smooth=double_smooth_path(&planner_map,raw_path,count,SAFE_SMOOTH_RAD,false,raw_path);
    int length=generate_safe_corner_path(&planner_map,raw_path,smooth,0.5f,0.05f,false,route);
    /* Upstream bounded buffers can truncate a long path: never call that arrival.
       Check the endpoint and every connecting segment; fall back to raw A*. */
    if(length<2 || hypotf(route[length-1].x-goal.x,route[length-1].y-goal.y)>0.001f ||
       !route_safe(route,(unsigned)length)) {
        length=(int)previous_len;
        memcpy(route,previous_path,previous_len*sizeof(Point2D));
    }
    if(length<2 || !route_safe(route,(unsigned)length)) return false;
    route_len=(unsigned)length;
    route_offset_x=offset_x; route_offset_y=offset_y;
    for(unsigned i=0;i<route_len;++i) {
        route[i].x-=planner_origin+route_offset_x;
        route[i].y-=planner_origin+route_offset_y;
    }
    return true;
}
bool W4Maze_LoadRoute(uint32_t id,const W4Pose *pose) {
    if(!W4Maze_Active(id)||!route_ready) return false;
    /* The path task cannot overwrite route until control requests a new plan. */
    W4Noetic_Load(route,route_len,pose);
    route_index=0; aim_valid=false;
    return true;
}
W4NoeticCommand W4Maze_Track(const W4Pose *pose,float speed,float dt) {
    W4NoeticCommand cmd=W4Noetic_Step(pose,speed,dt);
    uint32_t key=W4Port_Lock();
    track_aim=cmd.lookahead; aim_valid=cmd.valid; route_index=cmd.index;
    W4Port_Unlock(key);
    return cmd;
}
unsigned W4Maze_RouteCount(void) { return route_ready?route_len:0; }
/* A full route is retained between scans. Only a newly blocked route requests
   replanning; ordinary intermediate points never stop or restart the motors. */
static bool remaining_blocked(const W4Pose *pose) {
    unsigned first=route_index+1;
    Point2D a={pose->x_m+offset_x,pose->y_m+offset_y};
    float checked=0;
    for(unsigned i=first;i<route_len;++i) {
        Point2D b={route[i].x+offset_x,route[i].y+offset_y};
        float length=hypotf(b.x-a.x,b.y-a.y);
        int steps=1+(int)ceilf(length/0.025f);
        for(int j=0;j<=steps;++j) {
            float f=(float)j/steps;
            int index=index_at(a.x+f*(b.x-a.x),a.y+f*(b.y-a.y));
            if(index<0 || wall[index]) return true;
            /* Unknown space may guide A*, but is never executable near the car. */
            if(checked+f*length<0.28f && !free_cell(index)) return true;
        }
        checked+=length; a=b;
    }
    if(aim_valid && !clear_line(pose->x_m+offset_x,pose->y_m+offset_y,
                                track_aim.x+offset_x,track_aim.y+offset_y)) return true;
    return false;
}
''' + s[b:]
s=s.replace("offset_x=offset_y=0; reference_count=0; map_epoch=version;","offset_x=offset_y=0; reference_count=0; map_epoch=version;\n        previous_len=0; route_ready=false; aim_valid=false;")
a=s.index("        if(W4Maze_Active(n->cmd_id) && n->state==W4_TRACKING &&")
b=s.index("\n    }\n    if(W4Maze_Fresh",a)
s=s[:a]+'''        if(W4Maze_Active(n->cmd_id) && n->state==W4_TRACKING && route_ready &&
           remaining_blocked(&n->pose)) blocked=true;'''+s[b:]
a=s.index("    float nx=0,ny=0; bool last=false;")
s=s[:a]+'''    uint32_t now=W4Port_Millis();
    if(!ack_pending && now-plan_stamp<300U) return;
    bool valid=W4Maze_Fresh(now);
    bool ok=valid && plan(n->pose.x_m+offset_x,n->pose.y_m+offset_y,
                         n->target_x,n->target_y);
    key=W4Port_Lock();
    if(valid && !W4Maze_Fresh(W4Port_Millis())) { valid=false; ok=false; }
    if(!need_leg || w4_stop_pending || w4_nav.state!=W4_PLANNING ||
       w4_nav.cmd_id!=n->cmd_id || epoch!=version) { W4Port_Unlock(key); return; }
    if(!ok) {
        w4_nav.state=W4_FAULT; w4_nav.fault=valid?10:9; need_leg=false;
        W4Port_Unlock(key);
        W4Ack(n->cmd_id,"REJECTED",valid?"NO_CLEAR_PATH":"SCAN_STALE");
        return;
    }
    leg_x=route[route_len-1].x; leg_y=route[route_len-1].y;
    final_leg=true; blocked=false; route_index=0; aim_valid=false;
    route_ready=true; need_leg=false; plan_stamp=now; ++route_number;
    W4Port_Unlock(key);
    char line[96];
    int len=snprintf(line,sizeof line,"D,%lu,MAZE_ROUTE,%lu,%u,%u\\n",
        (unsigned long)now,(unsigned long)n->cmd_id,route_number,route_len);
    if(len>0 && len<(int)sizeof line) W4Telemetry_SubmitLine(line,(uint16_t)len);
    if(ack_pending) { W4Ack(n->cmd_id,"OK","MAZE_ACCEPTED"); ack_pending=false; }
}
#endif
'''
s=s.replace("static float offset_x,offset_y,leg_x,leg_y;","static float offset_x,offset_y;")
s=s.replace("    leg_x=route[route_len-1].x; leg_y=route[route_len-1].y;\n","")
write("week4/Src/w4_maze.c",s)
h=(R/"week4/Inc/w4_maze.h").read_text().replace('#include "w4_nav.h"','#include "w4_nav.h"\n#include "w4_noetic_motion.h"')
h=h.replace("#endif","bool W4Maze_LoadRoute(uint32_t id,const W4Pose *pose);\nW4NoeticCommand W4Maze_Track(const W4Pose *pose,float speed,float dt);\nunsigned W4Maze_RouteCount(void);\n#endif")
write("week4/Inc/w4_maze.h",h)
s=(R/"build/noetic_port/before/w4_control.c").read_text()
s=s.replace("static W4Pid heading_pid;","static W4Pid heading_pid;\n#ifdef W4_MAZE\nstatic bool maze_loaded;\n#endif")
s=s.replace("static void stop_control(void) {","static void stop_control(void) {\n#ifdef W4_MAZE\n    maze_loaded=false;\n    W4Noetic_Reset();\n#endif")
a=s.index("#ifdef W4_MAZE\n    bool maze=W4Maze_Active")
b=s.index("\n    if(!sensors &&",a)
s=s[:a]+'''#ifdef W4_MAZE
    bool maze=W4Maze_Active(n.cmd_id);
    if(maze) {
        if(!sensors) { fault(1); return; }
        if(!W4Maze_Fresh(now)) { fault(9); return; }
        if(W4Maze_LegBlocked()) {
            stop_control();
            if(n.state==W4_TRACKING && wheels_still(vl,vr)) {
                W4Maze_NextLeg(); state(W4_PLANNING,0);
            }
            /* Planning clears the latch only after publishing a valid new route. */
            if(n.state==W4_TRACKING) return;
        }
        if(!maze_loaded) {
            if(!W4Maze_LoadRoute(n.cmd_id,&n.pose)) {
                W4Motor_Stop();
                if(!missing) { missing=true; missing_since=now; }
                if(now-missing_since>2000U) fault(4);
                return;
            }
            maze_loaded=true; missing=false; settling_goal=false;
            state(W4_TRACKING,0);
        }
        float speed_limit=fminf(w4_config.max_v,0.18f);
#ifdef W4_REAR_FORWARD
        speed_limit=fminf(cruise(speed_limit,dt),0.18f);
#endif
        W4NoeticCommand command=W4Maze_Track(&n.pose,speed_limit,dt);
        if(!command.valid) { fault(11); return; }
        k=W4Port_Lock(); w4_nav.wp_index=(uint16_t)command.index; W4Port_Unlock(k);
        if(command.reached || settling_goal) {
            W4Motor_Stop();
            stall_time[0]=stall_time[1]=0;
            if(!settling_goal || !wheels_still(vl,vr)) arrival_since=now;
            settling_goal=true;
            if(now-arrival_since>=200U) {
                if(command.reached) { stop_control(); state(W4_ARRIVED,0); }
                else settling_goal=false;
            }
            return;
        }
        float turn=command.w*w4_config.track_m*0.5f;
        float left=command.v-turn,right=command.v+turn;
        float peak=fmaxf(fabsf(left),fabsf(right));
        if(peak>w4_config.max_wheel) {
            left*=w4_config.max_wheel/peak; right*=w4_config.max_wheel/peak;
        }
        drive_wheels(left,right,vl,vr,dt,now,begin,command.v>0);
        return;
    }
#endif''' + s[b:]
# Remove now unreachable legacy MAZE per-leg controller blocks.
s=re.sub(r"#ifdef W4_MAZE\n            if\(maze && !W4Maze_Fresh\(now\)\) \{ fault\(9\); return; \}\n#endif\n","",s)
a=s.index("#ifdef W4_MAZE\n    /* Short legs")
b=s.index("#endif",a)+len("#endif\n")
s=s[:a]+s[b:]
s=s.replace("#ifdef W4_MAZE\n    if(maze) speed_limit=fminf(speed_limit,0.18f);\n#endif\n","")
a=s.index("#ifdef W4_MAZE\n    /* Keep the lookahead")
b=s.index("#endif",a)+len("#endif\n")
s=s[:a]+s[b:]
s=s.replace("#ifdef W4_MAZE\n    if(maze) braking_speed=sqrtf(0.30f*fmaxf(0.0f,remaining-tol));\n#endif\n","")
a=s.index("#ifdef W4_MAZE\n    if(maze) {")
b=s.index("#endif",a)+len("#endif\n")
s=s[:a]+s[b:]
s=s.replace("    bool loaded_forward=false;\n#ifdef W4_MAZE\n    loaded_forward=maze && v>0;\n#endif\n    drive_wheels(l,r,vl,vr,dt,now,begin,loaded_forward);","    drive_wheels(l,r,vl,vr,dt,now,begin,false);")
write("week4/Src/w4_control.c",s)
# Existing glob-based firmware build automatically compiles the new modules.
p=R.parent/"run_cortex_tests.py"
s=p.read_text()
s=s.replace('"week4/Src/w4_maze.c"],','"week4/Src/w4_maze.c", *[f"week4/Src/w4_noetic_{name}.c" for name in ("planner_algo","priority_queue","path_smoother","motion")]],')
s=s.replace("    for name,sources,defines,sensor in cases:","    if len(sys.argv)>1:\n        cases=[case for case in cases if case[0] in sys.argv[1:]]\n        if not cases: raise SystemExit('No matching test cases')\n    for name,sources,defines,sensor in cases:")
p.write_text(s,encoding="utf-8",newline="\n")
print("Integrated full-route planner with MAZE and existing safety/motor interfaces.")
