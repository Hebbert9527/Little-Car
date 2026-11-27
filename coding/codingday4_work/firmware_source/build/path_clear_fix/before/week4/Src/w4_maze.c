#ifdef W4_MAZE
#include "w4_maze.h"
#include "w4_rtos.h"
#include "w4_noetic_motion.h"
#include "planner_algo.h"
#include "path_smoother.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

enum { WIDTH=MAX_GRID_SIZE, CENTER=WIDTH/2, CELLS=WIDTH*WIDTH, BEAMS=120, CLEARANCE=2 };
enum { NEAR_HIT_SCANS=3, FREE=1, OCCUPIED=2, COUNT_SHIFT=2, COUNT_MASK=12, HIT_THIS_SCAN=16 };
#define CELL_M PLANNER_MAP_RES
/* C1 Standard Scan angles increase clockwise viewed from above. */
#ifndef W4_C1_ANGLE_SIGN
#define W4_C1_ANGLE_SIGN (-1)
#endif
#ifndef W4_C1_FORWARD_OFFSET_M
#define W4_C1_FORWARD_OFFSET_M 0.0f
#endif
#ifndef W4_C1_LEFT_OFFSET_M
#define W4_C1_LEFT_OFFSET_M 0.0f
#endif
typedef struct { float ox,oy,x,y; bool hit,near; } Ray;
typedef struct { Ray ray[BEAMS]; unsigned count; uint32_t stamp,epoch; } Scan;
static Scan accumulating,published,working;
static bool synchronized,ready,ack_pending;
static volatile uint32_t epoch,scan_epoch,active,last_scan;
static volatile bool fresh,need_leg,blocked,final_leg;
/* One byte per cell: free/occupied flags, 2-bit confirmation count, scan hit. */
static uint8_t grid[CELLS];
static uint8_t changed_rows[(WIDTH+7)/8];
static float offset_x,offset_y;
static uint8_t wall[(CELLS+7)/8];
static bool wall_at(int i) { return (wall[i/8] & (1U<<(i%8)))!=0; }
static bool occupied(int i) { return (grid[i] & (OCCUPIED|COUNT_MASK))!=0; }
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

static Ray reference[BEAMS];
static unsigned reference_count;
static float ref_x,ref_y,ref_heading;

static int coordinate(float v) { return CENTER+(int)floorf(v/CELL_M+0.50001f); }
static int cell(int x,int y) {
    return x>=0 && x<WIDTH && y>=0 && y<WIDTH ? y*WIDTH+x : -1;
}
static int index_at(float x,float y) { return cell(coordinate(x),coordinate(y)); }
bool W4Maze_Active(uint32_t id) { return id && active==id; }
bool W4Maze_Fresh(uint32_t now) { return fresh && scan_epoch==epoch && now-last_scan<=450U; }
bool W4Maze_LegBlocked(void) { return blocked; }
bool W4Maze_FinalLeg(void) { return final_leg; }
void W4Maze_NextLeg(void) { route_ready=false; need_leg=true; }
void W4Maze_Begin(uint32_t id) { active=id; ack_pending=true; blocked=false; previous_len=0; route_number=0; W4Maze_NextLeg(); }
/* Called by the control task when the odometry origin changes. */
void W4Maze_Reset(void) { ++epoch; active=0; fresh=false; route_ready=false; aim_valid=false; }

void W4Maze_Point(float angle,float mm,unsigned quality,bool start,uint32_t now) {
    static uint32_t scan_start;
    if(start) {
        if(synchronized && accumulating.epoch==epoch && accumulating.count>=24 &&
           now-scan_start<=450U) {
            accumulating.stamp=now;
            uint32_t key=W4Port_Lock();
            published=accumulating;
            ready=true;
            W4Port_Unlock(key);
        }
        memset(&accumulating,0,sizeof accumulating);
        accumulating.epoch=epoch;
        scan_start=now;
        synchronized=true;
    }
    if(!synchronized || !quality || mm<50 || angle<0 || angle>=360) return;
    /* Bench telemetry showed a weak, repeating return near 0.33 m.
       Drop it before choosing a beam and do not clear space along it. */
    if(quality<=4 && mm>=300.0f && mm<=390.0f) return;
    unsigned bin=(unsigned)(angle/3.0f);
    Ray *r=&accumulating.ray[bin];
    if(r->hit || r->ox!=r->x || r->oy!=r->y) return;
    W4Pose p;
    if(!W4Pose_GetSnapshot(&p)||!p.valid||now-p.stamp_ms>50) return;
    float a=p.theta_rad+W4_C1_ANGLE_SIGN*angle*(W4_PI/180.0f);
    float distance=fminf(mm*0.001f,6.0f);
    float c=cosf(p.theta_rad),s=sinf(p.theta_rad);
    r->ox=p.x_m+c*W4_C1_FORWARD_OFFSET_M-s*W4_C1_LEFT_OFFSET_M;
    r->oy=p.y_m+s*W4_C1_FORWARD_OFFSET_M+c*W4_C1_LEFT_OFFSET_M;
    r->x=r->ox+distance*cosf(a); r->y=r->oy+distance*sinf(a);
    r->hit=mm<=6000;
    r->near=mm<=1000;
    ++accumulating.count;
}

/* Translation-only point-to-point ICP. IMU supplies yaw. Use a fixed
   reference until the chassis moves, so stationary scans cannot accumulate drift. */
static void register_scan(const W4Pose *p) {
    float tx=offset_x,ty=offset_y;
    for(unsigned iteration=0;iteration<3 && reference_count>=20;++iteration) {
        float dx=0,dy=0; unsigned pairs=0;
        for(unsigned j=0;j<BEAMS;++j) {
            Ray *r=&working.ray[j];
            if(!r->hit) continue;
            float x=r->x+tx,y=r->y+ty,best=0.08f*0.08f,bx=0,by=0;
            bool found=false;
            for(unsigned k=0;k<reference_count;++k) {
                float ex=reference[k].x-x,ey=reference[k].y-y,d=ex*ex+ey*ey;
                if(d<best) { best=d; bx=ex; by=ey; found=true; }
            }
            if(found) { dx+=bx; dy+=by; ++pairs; }
        }
        if(pairs<20 || pairs*3<working.count) break;
        tx+=dx/pairs; ty+=dy/pairs;
    }
    if(hypotf(tx-offset_x,ty-offset_y)<=0.04f) { offset_x=tx; offset_y=ty; }
    if(!reference_count || hypotf(p->x_m-ref_x,p->y_m-ref_y)>0.24f ||
       fabsf(W4Wrap(p->theta_rad-ref_heading))>0.45f) {
        reference_count=0;
        for(unsigned j=0;j<BEAMS;++j) if(working.ray[j].hit) {
            Ray *r=&reference[reference_count++];
            r->x=working.ray[j].x+offset_x; r->y=working.ray[j].y+offset_y;
        }
        ref_x=p->x_m; ref_y=p->y_m; ref_heading=p->theta_rad;
    }
}

static void trace(const Ray *r) {
    if(r->ox==r->x && r->oy==r->y) return;
    int x=coordinate(r->ox+offset_x),y=coordinate(r->oy+offset_y);
    int ex=coordinate(r->x+offset_x),ey=coordinate(r->y+offset_y);
    int dx=abs(ex-x),dy=-abs(ey-y),sx=x<ex?1:-1,sy=y<ey?1:-1,err=dx+dy;
    for(;;) {
        int i=cell(x,y);
        if(i<0) break;
        if(x==ex && y==ey) break;
        /* Static environment: confirmed walls stay occupied until RESET. */
        grid[i]|=FREE; /* OCCUPIED is never cleared by a ray. */
        int twice=2*err;
        if(twice>=dy) { err+=dy; x+=sx; }
        if(twice<=dx) { err+=dx; y+=sy; }
    }
}
/* Publish a rolling snapshot. A lost row is refreshed on the next sweep. */
static void publish_row(uint32_t version) {
    static unsigned row;
    static uint32_t sent;
    static unsigned priority_cursor;
    static bool priority_turn;
    uint32_t now=W4Port_Millis();
    if(now-sent<50U) return;
    unsigned selected=row;
    /* New walls get priority; alternate with a full sweep so dropped rows and
       reconnects are repaired even while new walls are being discovered. */
    if(priority_turn) for(unsigned j=0;j<WIDTH;++j) {
        unsigned candidate=(priority_cursor+j)%WIDTH;
        if(changed_rows[candidate/8] & (1U<<(candidate%8))) { selected=candidate; break; }
    }
    char line[192],cells[WIDTH+1];
    for(unsigned x=0;x<WIDTH;++x) {
        int value=grid[selected*WIDTH+x];
        cells[x]=(value&OCCUPIED)?'#':(value&FREE) && !(value&COUNT_MASK)?'.':'?';
    }
    cells[WIDTH]=0;
    /* Scan age is independent of transport age: cached rows do not make a
       stale scan usable by the controller or appear live on the PC. */
    uint32_t age=fresh && scan_epoch==version ? now-last_scan : UINT32_MAX;
    int len=snprintf(line,sizeof line,"M,%lu,%u,%ld,%ld,%u,%u,%lu,%s\n",
        (unsigned long)version,selected,lroundf(offset_x*1000),lroundf(offset_y*1000),
        (unsigned)lroundf(CELL_M*1000),CENTER,(unsigned long)age,cells);
    if(len>0 && len<(int)sizeof line && W4Telemetry_SubmitLine(line,(uint16_t)len)) {
        changed_rows[selected/8]&=(uint8_t)~(1U<<(selected%8));
        if(selected==row) row=(row+1)%WIDTH;
        priority_cursor=(selected+1)%WIDTH;
        priority_turn=!priority_turn;
    }
    sent=now;
}
static void inflate(void) {
    memset(wall,0,sizeof wall);
    for(int i=0;i<CELLS;++i) if(occupied(i)) {
        for(int dy=-CLEARANCE;dy<=CLEARANCE;++dy)
            for(int dx=-CLEARANCE;dx<=CLEARANCE;++dx) {
                if(dx*dx+dy*dy>CLEARANCE*CLEARANCE) continue;
                int n=cell(i%WIDTH+dx,i/WIDTH+dy);
                if(n>=0) wall[n/8]|=(uint8_t)(1U<<(n%8));
            }
    }
}
static bool free_cell(int i) {
    return i>=0 && !wall_at(i) && (grid[i]&FREE);
}

static bool clear_line(float x,float y,float gx,float gy) {
    int steps=1+(int)ceilf(hypotf(gx-x,gy-y)/(CELL_M*0.25f));
    int previous_x=coordinate(x), previous_y=coordinate(y);
    for(int j=0;j<=steps;++j) {
        float f=(float)j/steps;
        int cx=coordinate(x+f*(gx-x)), cy=coordinate(y+f*(gy-y));
        if(!free_cell(cell(cx,cy))) return false;
        if(cx!=previous_x && cy!=previous_y &&
           (!free_cell(cell(previous_x,cy)) || !free_cell(cell(cx,previous_y))))
            return false;
        previous_x=cx;
        previous_y=cy;
    }
    return true;
}

/* NoeticMaze uses positive, x-major planner coordinates; Week4 retains
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
        set_grid_val(&planner_map,col,row,occupied(i)?1:(grid[i]&FREE)?2:0);
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
    uint32_t key=W4Port_Lock();
    unsigned first=route_index+1;
    bool have_aim=aim_valid;
    Point2D aim=track_aim;
    W4Port_Unlock(key);
    Point2D a={pose->x_m+offset_x,pose->y_m+offset_y};
    float checked=0;
    for(unsigned i=first;i<route_len;++i) {
        Point2D b={route[i].x+offset_x,route[i].y+offset_y};
        float length=hypotf(b.x-a.x,b.y-a.y);
        int steps=1+(int)ceilf(length/0.025f);
        for(int j=0;j<=steps;++j) {
            float f=(float)j/steps;
            int index=index_at(a.x+f*(b.x-a.x),a.y+f*(b.y-a.y));
            if(index<0 || wall_at(index)) return true;
            /* Unknown space may guide A*, but is never executable near the car. */
            if(checked+f*length<0.28f && !free_cell(index)) return true;
        }
        checked+=length; a=b;
    }
    if(have_aim && !clear_line(pose->x_m+offset_x,pose->y_m+offset_y,
                                aim.x+offset_x,aim.y+offset_y)) return true;
    return false;
}
void W4Maze_Poll(const W4NavSnapshot *n) {
    static uint32_t map_epoch;
    uint32_t version=epoch;
    if(map_epoch!=version) {
        memset(grid,0,sizeof grid); memset(wall,0,sizeof wall);
        memset(changed_rows,0,sizeof changed_rows);
        offset_x=offset_y=0; reference_count=0; map_epoch=version;
        previous_len=0; route_ready=false; aim_valid=false;
    }
    uint32_t key=W4Port_Lock();
    bool got=ready;
    if(got) { working=published; ready=false; }
    W4Port_Unlock(key);
    if(got && working.epoch==version && n->pose.valid) {
        register_scan(&n->pose);
        for(unsigned i=0;i<BEAMS;++i) trace(&working.ray[i]);
        /* Count distinct revolutions, even when adjacent beams hit one cell. */
        for(unsigned j=0;j<BEAMS;++j) if(working.ray[j].hit) {
            const Ray *ray=&working.ray[j];
            int i=index_at(ray->x+offset_x,ray->y+offset_y);
            if(i<0) continue;
            if(!ray->near) grid[i]|=COUNT_MASK;
            grid[i]|=HIT_THIS_SCAN;
        }
        for(int i=0;i<CELLS;++i) {
            unsigned count=(grid[i]&COUNT_MASK)>>COUNT_SHIFT;
            bool hit=(grid[i]&HIT_THIS_SCAN)!=0;
            grid[i]&=(FREE|OCCUPIED);
            if(!hit) continue;
            if(count<NEAR_HIT_SCANS) ++count;
            grid[i]|=(uint8_t)(count<<COUNT_SHIFT);
            if(count==NEAR_HIT_SCANS && !(grid[i]&OCCUPIED)) {
                grid[i]|=OCCUPIED;
                unsigned row=(unsigned)i/WIDTH;
                changed_rows[row/8]|=(uint8_t)(1U<<(row%8));
            }
        }
        inflate();
        scan_epoch=version; last_scan=working.stamp; fresh=true;
        if(W4Maze_Active(n->cmd_id) && n->state==W4_TRACKING && route_ready &&
           remaining_blocked(&n->pose)) blocked=true;
    }
    /* Keep serving the persistent map even while IMU/LiDAR are recovering. */
    publish_row(version);
    static uint32_t last_marker;
    uint32_t marker_now=W4Port_Millis();
    if(n->state==W4_TRACKING && W4Maze_Active(n->cmd_id) && marker_now-last_marker>=100U) {
        key=W4Port_Lock();
        Point2D aim=track_aim; bool have_aim=aim_valid;
        W4Port_Unlock(key);
        if(have_aim) {
            char line[96];
            int len=snprintf(line,sizeof line,"D,%lu,MAZE_LEG,%lu,%ld,%ld\n",
                (unsigned long)marker_now,(unsigned long)n->cmd_id,
                lroundf(aim.x*1000),lroundf(aim.y*1000));
            if(len>0 && len<(int)sizeof line) W4Telemetry_SubmitLine(line,(uint16_t)len);
        }
        last_marker=marker_now;
    }
    if(!W4Maze_Active(n->cmd_id)||n->state!=W4_PLANNING||!need_leg) return;
    uint32_t now=W4Port_Millis();
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
    final_leg=true; blocked=false; route_index=0; aim_valid=false;
    route_ready=true; need_leg=false; plan_stamp=now; ++route_number;
    W4Port_Unlock(key);
    char line[96];
    int len=snprintf(line,sizeof line,"D,%lu,MAZE_ROUTE,%lu,%u,%u\n",
        (unsigned long)now,(unsigned long)n->cmd_id,route_number,route_len);
    if(len>0 && len<(int)sizeof line) W4Telemetry_SubmitLine(line,(uint16_t)len);
    if(ack_pending) { W4Ack(n->cmd_id,"OK","MAZE_ACCEPTED"); ack_pending=false; }
}
#endif
