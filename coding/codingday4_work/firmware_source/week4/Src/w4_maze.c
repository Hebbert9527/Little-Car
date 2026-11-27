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

enum { WIDTH=MAX_GRID_SIZE, CENTER=WIDTH/2, CELLS=WIDTH*WIDTH, BEAMS=120 };
enum {
    HIT_CONFIRM_SCANS=3, FREE=1U, OCCUPIED=2U,
    SCORE_SHIFT=2, SCORE_MAX=12U, SCORE_MASK=(15U<<SCORE_SHIFT),
    HIT_STEP=3U, CLEAR_STEP=3U,
    CONFIRM_SCORE=(HIT_CONFIRM_SCANS*HIT_STEP), CLEAR_SCORE=3U,
    PASS_THIS_SCAN=64U, HIT_THIS_SCAN=128U
};
enum { TRAIL_MAX=256 };
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
typedef struct { float ox,oy,x,y; bool hit; } Ray;
typedef struct { Ray ray[BEAMS]; unsigned count; uint32_t stamp,epoch; } Scan;
static Scan accumulating,published,working;
static bool synchronized,ready,ack_pending;
static volatile uint32_t epoch,scan_epoch,active,last_scan;
static volatile bool fresh,need_leg,immediate_blocked,final_leg,returning_mode;
static volatile uint32_t route_seq;
/* One byte per cell: free/occupied flags, 2-bit confirmation count, scan hit. */
static uint8_t grid[CELLS];
static uint8_t changed_rows[(WIDTH+7)/8];
static float offset_x,offset_y;
static uint8_t wall[(CELLS+7)/8];
static bool wall_at(int i) { return (wall[i/8] & (1U<<(i%8)))!=0; }
/* Only confirmed walls block planning so single-frame noise does not stall the car. */
static bool occupied(int i) { return (grid[i] & OCCUPIED)!=0; }
PlannerMemoryPool g_mem_pool;
static uint8_t planner_grid[MAX_MAP_BYTES];
static ServerMap planner_map={planner_grid,CELL_M,WIDTH};
static Point2D raw_path[MAX_PATH_LEN],previous_path[MAX_PATH_LEN],route[MAX_PATH_LEN];
static unsigned previous_len,route_len;
static volatile bool route_ready;
static uint32_t plan_stamp,last_plan_scan;
static bool waiting_for_route;
static uint32_t wait_report_stamp;
static uint8_t blocked_streak;
static const char *plan_failure="NO_ROUTE";
static volatile unsigned route_index;
static Point2D track_aim;
static bool aim_valid;
static float route_offset_x,route_offset_y;
static unsigned route_number;

/* Compact ICP reference cloud (Point2D saves 1.44 KB vs full Ray structs). */
static Point2D reference[BEAMS];
static unsigned reference_count;
static float ref_x,ref_y,ref_heading;
static float prev_scan_x,prev_scan_y,prev_scan_heading;
static bool have_prev_scan_pose;

/* Traversed breadcrumb trail in mm (1 KB total) for guaranteed RETURN fallback. */
static int16_t trail_x[TRAIL_MAX],trail_y[TRAIL_MAX];
static unsigned trail_len;

static int coordinate(float v) { return CENTER+(int)floorf(v/CELL_M+0.50001f); }
static int cell(int x,int y) {
    return x>=0 && x<WIDTH && y>=0 && y<WIDTH ? y*WIDTH+x : -1;
}
static int index_at(float x,float y) { return cell(coordinate(x),coordinate(y)); }
bool W4Maze_Active(uint32_t id) { return id && active==id; }
bool W4Maze_Returning(void) { return returning_mode; }
bool W4Maze_Fresh(uint32_t now) { return fresh && scan_epoch==epoch && now-last_scan<=450U; }
bool W4Maze_ImmediateBlocked(void) { return immediate_blocked; }
bool W4Maze_FinalLeg(void) { return final_leg; }
void W4Maze_RequestReplan(void) { need_leg=true; }
unsigned W4Maze_TrailCount(void) { return trail_len; }

static void trail_reset(void) {
    trail_x[0]=0; trail_y[0]=0; trail_len=1;
}
static void trail_record(float x_m,float y_m) {
    if(!isfinite(x_m)||!isfinite(y_m)) return;
    /* Mark the robot's traversed cell as observed free (unless confirmed wall)
       so the return corridor back to (0,0) remains connected. */
    int ci=index_at(x_m+offset_x,y_m+offset_y);
    if(ci>=0 && !(grid[ci]&OCCUPIED)) grid[ci]|=FREE;
    if(trail_len==0) trail_reset();
    int32_t xm=lroundf(x_m*1000.0f), ym=lroundf(y_m*1000.0f);
    if(xm<-30000||xm>30000||ym<-30000||ym>30000) return;
    int32_t dx=xm-trail_x[trail_len-1], dy=ym-trail_y[trail_len-1];
    if(dx*dx+dy*dy < 100*100) return;
    if(trail_len>=TRAIL_MAX) {
        /* Compact by keeping origin and every 2nd breadcrumb. */
        unsigned w=1;
        for(unsigned r=2;r<trail_len;r+=2) {
            trail_x[w]=trail_x[r]; trail_y[w]=trail_y[r]; ++w;
        }
        trail_len=w;
    }
    trail_x[trail_len]=(int16_t)xm;
    trail_y[trail_len]=(int16_t)ym;
    ++trail_len;
}

void W4Maze_Begin(uint32_t id,bool returning) {
    active=id; returning_mode=returning; ack_pending=true;
    immediate_blocked=false; blocked_streak=0; previous_len=0; route_number=0;
    waiting_for_route=false; route_ready=false; need_leg=true;
    plan_stamp=W4Port_Millis()-300U; last_plan_scan=UINT32_MAX;
}
/* Called by the control task when the odometry origin changes. */
void W4Maze_Reset(void) {
    ++epoch; active=0; fresh=false; route_ready=false; aim_valid=false;
    immediate_blocked=false; returning_mode=false; blocked_streak=0;
}

void W4Maze_Point(float angle,float mm,unsigned quality,bool start,uint32_t now) {
    static uint32_t scan_start;
    if(start) {
        if(synchronized && accumulating.epoch==epoch && accumulating.count>=24 &&
           now-scan_start<=600U) {
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
    if(!W4Pose_GetSnapshot(&p)||!p.valid||now-p.stamp_ms>200) return;
    float a=p.theta_rad+W4_C1_ANGLE_SIGN*angle*(W4_PI/180.0f);
    float distance=fminf(mm*0.001f,6.0f);
    float c=cosf(p.theta_rad),s=sinf(p.theta_rad);
    r->ox=p.x_m+c*W4_C1_FORWARD_OFFSET_M-s*W4_C1_LEFT_OFFSET_M;
    r->oy=p.y_m+s*W4_C1_FORWARD_OFFSET_M+c*W4_C1_LEFT_OFFSET_M;
    r->x=r->ox+distance*cosf(a); r->y=r->oy+distance*sinf(a);
    r->hit=mm<=6000;
    ++accumulating.count;
}

/* Normal-projected (point-to-line) translation ICP with ZUPT and turn guard.
   Prevents wall-tangent sliding, stationary creep, and runaway offset drift. */
static void register_scan(const W4Pose *p) {
    if(!reference_count) {
        for(unsigned j=0;j<BEAMS;++j) if(working.ray[j].hit) {
            Point2D *r=&reference[reference_count++];
            r->x=working.ray[j].x+offset_x; r->y=working.ray[j].y+offset_y;
        }
        ref_x=prev_scan_x=p->x_m; ref_y=prev_scan_y=p->y_m;
        ref_heading=prev_scan_heading=p->theta_rad;
        have_prev_scan_pose=reference_count>0;
        return;
    }
    if(have_prev_scan_pose) {
        float step_d=hypotf(p->x_m-prev_scan_x,p->y_m-prev_scan_y);
        float step_a=fabsf(W4Wrap(p->theta_rad-prev_scan_heading));
        prev_scan_x=p->x_m; prev_scan_y=p->y_m; prev_scan_heading=p->theta_rad;
        if((step_d<0.003f && step_a<0.006f) || step_a>0.035f) return;
    } else {
        prev_scan_x=p->x_m; prev_scan_y=p->y_m; prev_scan_heading=p->theta_rad;
        have_prev_scan_pose=true;
    }
    float tx=offset_x,ty=offset_y;
    for(unsigned iteration=0;iteration<3 && reference_count>=20;++iteration) {
        float a00=0,a01=0,a11=0,b0=0,b1=0;
        unsigned pairs=0;
        for(unsigned j=0;j<BEAMS;++j) {
            Ray *r=&working.ray[j];
            if(!r->hit) continue;
            float x=r->x+tx,y=r->y+ty,best=0.07f*0.07f;
            int best_k=-1;
            for(unsigned k=0;k<reference_count;++k) {
                float ex=reference[k].x-x,ey=reference[k].y-y,d=ex*ex+ey*ey;
                if(d<best) { best=d; best_k=(int)k; }
            }
            if(best_k<0) continue;
            unsigned k0=(best_k>0)?(unsigned)(best_k-1):(unsigned)best_k;
            unsigned k1=((unsigned)(best_k+1)<reference_count)?(unsigned)(best_k+1):(unsigned)best_k;
            float wx=reference[k1].x-reference[k0].x, wy=reference[k1].y-reference[k0].y;
            float wlen=hypotf(wx,wy);
            if(k0==k1 || wlen<0.02f || wlen>0.35f) continue;
            float nx=-wy/wlen, ny=wx/wlen;
            float err=(reference[best_k].x-x)*nx + (reference[best_k].y-y)*ny;
            a00+=nx*nx; a01+=nx*ny; a11+=ny*ny;
            b0+=nx*err; b1+=ny*err;
            ++pairs;
        }
        float det=a00*a11-a01*a01;
        if(pairs<16 || det<4.0f) break;
        float dtx=(a11*b0-a01*b1)/det;
        float dty=(a00*b1-a01*b0)/det;
        tx+=0.5f*dtx; ty+=0.5f*dty;
    }
    if(hypotf(tx-offset_x,ty-offset_y)<=0.025f) {
        offset_x=fmaxf(-0.04f,fminf(0.04f,tx));
        offset_y=fmaxf(-0.04f,fminf(0.04f,ty));
    }
    if(hypotf(p->x_m-ref_x,p->y_m-ref_y)>0.24f ||
       fabsf(W4Wrap(p->theta_rad-ref_heading))>0.45f) {
        reference_count=0;
        for(unsigned j=0;j<BEAMS;++j) if(working.ray[j].hit) {
            Point2D *r=&reference[reference_count++];
            r->x=working.ray[j].x+offset_x; r->y=working.ray[j].y+offset_y;
        }
        ref_x=p->x_m; ref_y=p->y_m; ref_heading=p->theta_rad;
    }
}

static void trace(const Ray *r) {
    if(r->ox==r->x && r->oy==r->y) return;
    int x=coordinate(r->ox+offset_x),y=coordinate(r->oy+offset_y);
    int ex=coordinate(r->x+offset_x),ey=coordinate(r->y+offset_y);
    int span_sq=(ex-x)*(ex-x)+(ey-y)*(ey-y);
    bool near_ray=!r->hit || span_sq<=15*15;
    int dx=abs(ex-x),dy=-abs(ey-y),sx=x<ex?1:-1,sy=y<ey?1:-1,err=dx+dy;
    for(;;) {
        int i=cell(x,y);
        if(i<0) break;
        if(x==ex && y==ey) break;
        grid[i]|=FREE;
        /* Mark pass-through for ghost-wall erosion; for distant (>1.5 m) hits,
           stop 1 cell before the endpoint so 3-deg grazing beams never poke holes. */
        if(near_ray || abs(x-ex)>1 || abs(y-ey)>1)
            grid[i]|=PASS_THIS_SCAN;
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
        cells[x]=(value&OCCUPIED)?'#':(value&FREE)?'.':'?';
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
/* Publish downsampled planned polyline so PC frontend displays full A* / return path. */
static void publish_path_telemetry(uint32_t now,uint32_t cmd_id) {
    if(!route_ready || route_len<2) return;
    char line[192];
    int pos=snprintf(line,sizeof line,"D,%lu,MAZE_PATH,%lu,%u",
        (unsigned long)now,(unsigned long)cmd_id,returning_mode?1U:0U);
    if(pos<=0 || pos>=(int)sizeof line) return;
    enum { MAX_TX_PTS=12 };
    unsigned count=route_len<MAX_TX_PTS?route_len:MAX_TX_PTS;
    for(unsigned s=0;s<count;++s) {
        unsigned idx=(s+1==count)?(route_len-1U):(s*(route_len-1U))/(count-1U);
        int added=snprintf(line+pos,sizeof line-(size_t)pos,",%ld,%ld",
            lroundf(route[idx].x*1000.0f),lroundf(route[idx].y*1000.0f));
        if(added<=0 || pos+added+2>=(int)sizeof line) break;
        pos+=added;
    }
    line[pos++]='\n'; line[pos]=0;
    (void)W4Telemetry_SubmitLine(line,(uint16_t)pos);
}
static void inflate(void) {
    float r_g=PHYSICAL_RADIUS/CELL_M;
    int r_sq=(int)floorf(r_g*r_g+1e-4f);
    int r_lim=(r_g>1e-4f)?(int)floorf(r_g+1e-4f):0;
    while((r_lim+1)*(r_lim+1)<=r_sq) ++r_lim;
    while(r_lim>0 && r_lim*r_lim>r_sq) --r_lim;
    memset(wall,0,sizeof wall);
    for(int i=0;i<CELLS;++i) if(occupied(i)) {
        for(int dy=-r_lim;dy<=r_lim;++dy)
            for(int dx=-r_lim;dx<=r_lim;++dx) {
                if(dx*dx+dy*dy>r_sq) continue;
                int n=cell(i%WIDTH+dx,i/WIDTH+dy);
                if(n>=0) wall[n/8]|=(uint8_t)(1U<<(n%8));
            }
    }
}
static bool free_cell(int i) {
    return i>=0 && !wall_at(i) && (grid[i]&FREE);
}

/* NoeticMaze uses positive, x-major planner coordinates; Week4 retains
   its centered y-major telemetry map and +Y initial heading. */
static const float planner_origin=(CENTER+0.5f)*CELL_M;
static bool collision_line(Point2D a,Point2D b,bool is_return,float ignore_start) {
    return bresenham_line_check(&planner_map,a,b,PHYSICAL_RADIUS,ignore_start,is_return);
}
static bool route_safe(const Point2D *p,unsigned count,bool is_return,bool allow_start_escape) {
    if(count<2 || count>MAX_PATH_LEN) return false;
    for(unsigned i=1;i<count;++i) {
        if(!isfinite(p[i].x)||!isfinite(p[i].y)) return false;
        float ignore=(allow_start_escape && i==1)?0.25f:0.0f;
        if(collision_line(p[i-1],p[i],is_return,ignore)) return false;
    }
    return true;
}
static void commit_route(const Point2D *src,unsigned length) {
    /* Seqlock write: odd while updating route[], even when published.
       W4ControlTask (higher priority) skips load on odd and hot-swaps on even. */
    route_seq=route_seq+1U;
    memcpy(route,src,(size_t)length*sizeof(Point2D));
    route_len=length;
    route_seq=route_seq+1U;
}
static bool plan(float x,float y,float gx,float gy,bool is_return) {
    memset(planner_grid,0,sizeof planner_grid);
    for(int row=0;row<WIDTH;++row) for(int col=0;col<WIDTH;++col) {
        int i=cell(col,row);
        set_grid_val(&planner_map,col,row,occupied(i)?1:(grid[i]&FREE)?2:0);
    }
    Point2D start={x+planner_origin,y+planner_origin};
    Point2D goal={gx+planner_origin,gy+planner_origin};
    bool start_grazing=check_collision(&planner_map,start,PHYSICAL_RADIUS,is_return);
    int count=astar_plan(&planner_map,start,goal,previous_path,(int)previous_len,is_return,raw_path);
    if(count<2) { plan_failure=W4Planner_FailureReason(); return false; }
    if(!route_safe(raw_path,(unsigned)count,is_return,start_grazing)) {
        plan_failure="PATH_COLLISION"; return false;
    }
    memcpy(previous_path,raw_path,(size_t)count*sizeof(Point2D));
    previous_len=(unsigned)count;
    int smooth=double_smooth_path(&planner_map,raw_path,count,SAFE_SMOOTH_RAD,is_return,raw_path);
    /* generate_safe_corner_path uses temp_bezier/temp_line, leaving path_buffer_B free. */
    Point2D *cand=g_mem_pool.smooth.path_buffer_B;
    int length=generate_safe_corner_path(&planner_map,raw_path,smooth,0.5f,0.05f,is_return,cand);
    /* Upstream bounded buffers can truncate a long path: never call that arrival.
       Check the endpoint and every connecting segment; fall back to raw A*. */
    if(length<2 || hypotf(cand[length-1].x-goal.x,cand[length-1].y-goal.y)>0.001f ||
       !route_safe(cand,(unsigned)length,is_return,start_grazing)) {
        length=(int)previous_len;
        cand=previous_path;
    }
    if(length<2 || !route_safe(cand,(unsigned)length,is_return,start_grazing)) {
        plan_failure="PATH_COLLISION"; return false;
    }
    route_offset_x=offset_x; route_offset_y=offset_y;
    for(int i=0;i<length;++i) {
        raw_path[i].x=cand[i].x-(planner_origin+route_offset_x);
        raw_path[i].y=cand[i].y-(planner_origin+route_offset_y);
    }
    commit_route(raw_path,(unsigned)length);
    return true;
}
/* Fallback for RETURN if tight maze walls prevent inflated A*: reverse breadcrumbs to (0,0). */
static bool build_trail_return_route(float cur_x,float cur_y) {
    if(trail_len==0) trail_reset();
    unsigned best_idx=trail_len-1U;
    float best_d2=1e9f;
    for(unsigned i=0;i<trail_len;++i) {
        float dx=cur_x-trail_x[i]*0.001f, dy=cur_y-trail_y[i]*0.001f;
        float d2=dx*dx+dy*dy;
        if(d2<best_d2) { best_d2=d2; best_idx=i; }
    }
    unsigned out_len=0;
    raw_path[out_len++]=(Point2D){cur_x,cur_y};
    for(int i=(int)best_idx;i>=0 && out_len<MAX_PATH_LEN-1;--i) {
        Point2D next={trail_x[i]*0.001f,trail_y[i]*0.001f};
        Point2D prev=raw_path[out_len-1];
        float d=hypotf(next.x-prev.x,next.y-prev.y);
        if(d<0.02f && i>0) continue;
        if(d>0.10f) {
            int steps=(int)ceilf(d/0.05f);
            for(int s=1;s<steps && out_len<MAX_PATH_LEN-1;++s) {
                float f=(float)s/(float)steps;
                raw_path[out_len++]=(Point2D){prev.x+f*(next.x-prev.x),prev.y+f*(next.y-prev.y)};
            }
        }
        if(out_len<MAX_PATH_LEN) raw_path[out_len++]=next;
    }
    if(out_len<2) {
        raw_path[out_len++]=(Point2D){0.0f,0.0f};
    }
    commit_route(raw_path,out_len);
    return true;
}
bool W4Maze_HasNewRoute(uint32_t id,uint32_t loaded_seq) {
    uint32_t seq=route_seq;
    return W4Maze_Active(id) && route_ready && (seq & 1U)==0U && seq!=loaded_seq;
}
bool W4Maze_LoadRoute(uint32_t id,const W4Pose *pose,bool hot_swap,uint32_t *loaded_seq) {
    if(!W4Maze_Active(id)||!route_ready) return false;
    uint32_t seq=route_seq;
    if((seq & 1U)!=0U) return false;
    if(hot_swap) W4Noetic_Swap(route,route_len,pose);
    else W4Noetic_Load(route,route_len,pose);
    if(route_seq!=seq) return false;
    if(loaded_seq) *loaded_seq=seq;
    route_index=W4Noetic_Index();
    aim_valid=false;
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

/* Match NoeticMaze: only confirmed inflated walls block forward exploration;
   unknown space does NOT stop the car. During RETURN, unknown space also
   triggers background replanning to stay inside explored corridors. */
static bool remaining_blocked(const W4Pose *pose,bool *near_block) {
    *near_block=false;
    uint32_t key=W4Port_Lock();
    unsigned first=route_index+1;
    W4Port_Unlock(key);
    Point2D a={pose->x_m+offset_x,pose->y_m+offset_y};
    float checked=0.0f;
    float check_horizon=returning_mode?3.0f:1.20f;
    for(unsigned i=first;i<route_len;++i) {
        Point2D b={route[i].x+offset_x,route[i].y+offset_y};
        float length=hypotf(b.x-a.x,b.y-a.y);
        int steps=1+(int)ceilf(length/0.025f);
        for(int j=0;j<=steps;++j) {
            float f=(float)j/steps;
            float dist=checked+f*length;
            if(dist>check_horizon) break;
            int index=index_at(a.x+f*(b.x-a.x),a.y+f*(b.y-a.y));
            if(index<0 || wall_at(index)) {
                if(dist<0.32f) *near_block=true;
                return true;
            }
            if(returning_mode && !free_cell(index)) return true;
        }
        checked+=length; a=b;
        if(checked>=check_horizon) break;
    }
    return false;
}
void W4Maze_Poll(const W4NavSnapshot *n) {
    static uint32_t map_epoch;
    static uint32_t last_path_tx;
    uint32_t version=epoch;
    if(map_epoch!=version) {
        memset(grid,0,sizeof grid); memset(wall,0,sizeof wall);
        memset(changed_rows,0,sizeof changed_rows);
        offset_x=offset_y=0; reference_count=0; have_prev_scan_pose=false; map_epoch=version;
        previous_len=0; route_ready=false; aim_valid=false;
        immediate_blocked=false; blocked_streak=0;
        trail_reset();
    }
    if(n->pose.valid) {
        trail_record(n->pose.x_m,n->pose.y_m);
    }
    uint32_t key=W4Port_Lock();
    bool got=ready;
    if(got) { working=published; ready=false; }
    W4Port_Unlock(key);
    if(got && working.epoch==version && n->pose.valid) {
        register_scan(&n->pose);
        for(unsigned i=0;i<BEAMS;++i) trace(&working.ray[i]);
        /* Per-revolution log-odds occupancy with pass-through ghost erosion:
           - 3 consecutive hit scans confirm '#' (saturating at SCORE_MAX);
           - 3 consecutive pass-through scans (with 0 hits in that cell) erase
             drifted '#' layers back to '.', keeping walls 1-cell thin;
           - occluded walls (neither hit nor passed through) stay '#' forever. */
        for(unsigned j=0;j<BEAMS;++j) if(working.ray[j].hit) {
            const Ray *ray=&working.ray[j];
            int i=index_at(ray->x+offset_x,ray->y+offset_y);
            if(i<0) continue;
            grid[i]|=HIT_THIS_SCAN;
        }
        for(int i=0;i<CELLS;++i) {
            unsigned score=(grid[i]&SCORE_MASK)>>SCORE_SHIFT;
            bool hit=(grid[i]&HIT_THIS_SCAN)!=0;
            bool pass=(grid[i]&PASS_THIS_SCAN)!=0;
            bool was_occ=(grid[i]&OCCUPIED)!=0;
            grid[i]&=(FREE|OCCUPIED);
            if(hit) {
                score=(score+HIT_STEP>SCORE_MAX)?SCORE_MAX:(score+HIT_STEP);
                if(score>=CONFIRM_SCORE && !was_occ) {
                    grid[i]|=OCCUPIED;
                    score=SCORE_MAX;
                    unsigned row=(unsigned)i/WIDTH;
                    changed_rows[row/8]|=(uint8_t)(1U<<(row%8));
                }
            } else if(was_occ) {
                if(pass) {
                    score=(score>CLEAR_STEP)?(score-CLEAR_STEP):0U;
                    if(score<=CLEAR_SCORE) {
                        grid[i]=(uint8_t)((grid[i]&~OCCUPIED)|FREE);
                        score=0U;
                        unsigned row=(unsigned)i/WIDTH;
                        changed_rows[row/8]|=(uint8_t)(1U<<(row%8));
                    }
                }
            } else {
                score=0U;
            }
            grid[i]|=(uint8_t)(score<<SCORE_SHIFT);
        }
        inflate();
        scan_epoch=version; last_scan=working.stamp; fresh=true;
        if(W4Maze_Active(n->cmd_id) && n->state==W4_TRACKING && route_ready) {
            bool near_block=false;
            if(remaining_blocked(&n->pose,&near_block)) {
                if(blocked_streak<255) ++blocked_streak;
                if(near_block) {
                    /* Confirmed wall immediately ahead: pause until detour is ready. */
                    immediate_blocked=true;
                    need_leg=true;
                } else if(blocked_streak>=2) {
                    /* Distant wall on route: replan in background while keeping moving! */
                    need_leg=true;
                }
            } else {
                blocked_streak=0;
                if(W4Noetic_OffPath()) need_leg=true;
            }
        }
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
    if(W4Maze_Active(n->cmd_id) && route_ready &&
       (n->state==W4_TRACKING || n->state==W4_PLANNING) && marker_now-last_path_tx>=1000U) {
        publish_path_telemetry(marker_now,n->cmd_id);
        last_path_tx=marker_now;
    }
    if(!W4Maze_Active(n->cmd_id) || (n->state!=W4_PLANNING && n->state!=W4_TRACKING) || !need_leg) return;
    uint32_t now=W4Port_Millis();
    if(now-plan_stamp<300U) return;
    if(waiting_for_route && fresh && last_plan_scan==last_scan) return;
    plan_stamp=now; last_plan_scan=last_scan;
    /* Match NoeticMaze: never fail with FAULT 9 when a scan is delayed. Plan on
       the latest map (or breadcrumb fallback for RETURN) and continuously refine. */
    bool ok=false;
    if(returning_mode) {
        ok=plan(n->pose.x_m+offset_x,n->pose.y_m+offset_y,offset_x,offset_y,true);
        if(!ok) ok=plan(n->pose.x_m+offset_x,n->pose.y_m+offset_y,offset_x,offset_y,false);
        if(!ok) ok=build_trail_return_route(n->pose.x_m,n->pose.y_m);
    } else {
        ok=plan(n->pose.x_m+offset_x,n->pose.y_m+offset_y,
                n->target_x+offset_x,n->target_y+offset_y,false);
    }
    key=W4Port_Lock();
    if(!need_leg || w4_stop_pending ||
       (w4_nav.state!=W4_PLANNING && w4_nav.state!=W4_TRACKING) ||
       w4_nav.cmd_id!=n->cmd_id || epoch!=version) { W4Port_Unlock(key); return; }
    if(!ok) {
        /* Never terminate with FAULT: wait & retry in background on new scans. */
        bool report=!waiting_for_route || now-wait_report_stamp>=1000U;
        waiting_for_route=true;
        if(w4_nav.state==W4_PLANNING || immediate_blocked) {
            route_ready=false;
            w4_nav.state=W4_PLANNING; w4_nav.fault=0;
        }
        W4Port_Unlock(key);
        if(report) {
            wait_report_stamp=now;
            W4Ack(n->cmd_id,"WAIT",plan_failure?plan_failure:"NO_ROUTE");
        }
        return;
    }
    bool resumed=waiting_for_route;
    waiting_for_route=false;
    final_leg=true; immediate_blocked=false; blocked_streak=0;
    route_ready=true; need_leg=false; plan_stamp=now; ++route_number;
    W4Port_Unlock(key);
    char line[96];
    int len=snprintf(line,sizeof line,"D,%lu,MAZE_ROUTE,%lu,%u,%u\n",
        (unsigned long)now,(unsigned long)n->cmd_id,route_number,route_len);
    if(len>0 && len<(int)sizeof line) W4Telemetry_SubmitLine(line,(uint16_t)len);
    publish_path_telemetry(now,n->cmd_id);
    last_path_tx=now;
    if(ack_pending) {
        W4Ack(n->cmd_id,"OK",returning_mode?"RETURN_ACCEPTED":"MAZE_ACCEPTED");
        ack_pending=false;
    } else if(resumed) {
        W4Ack(n->cmd_id,"OK",returning_mode?"RETURN_RESUMED":"MAZE_RESUMED");
    }
}
#endif
