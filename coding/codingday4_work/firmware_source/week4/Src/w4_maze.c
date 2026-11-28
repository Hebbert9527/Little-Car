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

enum { WIDTH=MAX_GRID_SIZE, CENTER=WIDTH/2, CELLS=WIDTH*WIDTH, BEAMS=240 };
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
/* Optional constant LiDAR range bias subtraction (metres). Keep 0.0f until
   confirmed with a ruler; field logs showed ~+0.23 m inward wall bowing. */
#ifndef W4_LIDAR_RANGE_OFFSET_M
#define W4_LIDAR_RANGE_OFFSET_M 0.0f
#endif
enum { RAY_EMPTY=-32768 };
typedef struct { int16_t ox_mm,oy_mm,x_mm,y_mm; } Ray;
typedef struct {
    Ray ray[BEAMS];
    uint8_t hit[(BEAMS+7)/8];
    unsigned count;
    uint32_t stamp,epoch;
} Scan;
static bool ray_hit(const Scan *s,unsigned b) { return (s->hit[b/8] & (1U<<(b%8)))!=0; }
static void ray_set_hit(Scan *s,unsigned b) { s->hit[b/8]|=(uint8_t)(1U<<(b%8)); }
static bool ray_valid(const Ray *r) { return r->ox_mm!=RAY_EMPTY; }
static Scan accumulating,published,working;
static bool synchronized,ready,ack_pending;
static volatile uint32_t epoch,scan_epoch,active,last_scan;
static volatile bool fresh,need_leg,immediate_blocked,final_leg,returning_mode;
static volatile uint32_t route_seq;
/* Persistent 5 cm map and planner share packed evidence; no duplicated map. */
static uint8_t grid[MAX_MAP_BYTES];
static ServerMap planner_map={grid,CELL_M,WIDTH};
static uint8_t changed_rows[(WIDTH+7)/8];
static float offset_x,offset_y;
static bool occupied(int i) { return map_state(&planner_map,i)>=4; }
static bool observed_free(int i) { return map_state(&planner_map,i)==1; }
PlannerMemoryPool g_mem_pool;
/* Two transient bits/cell, overlaid with A* nodes between searches. */
#define scan_flags ((uint8_t *)w4_planner_nodes)
_Static_assert((CELLS+3)/4<=sizeof w4_planner_nodes,"scan scratch capacity");
static void scan_mark(int i,unsigned flag) { scan_flags[i/4]|=flag<<((i%4)*2); }
static Point2D raw_path[MAX_PATH_LEN],previous_path[MAX_PATH_LEN],route[MAX_PATH_LEN];
static unsigned previous_len,route_len;
static volatile bool route_ready;
static bool route_is_trail_fallback;
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

/* Compact ICP reference cloud in mm (4 bytes/point, fits 240 beams in 960 B). */
typedef struct { int16_t x_mm,y_mm; } PointMm;
static PointMm reference[BEAMS];
static unsigned reference_count;
static float ref_x,ref_y,ref_heading;
static float prev_scan_x,prev_scan_y,prev_scan_heading;
static bool have_prev_scan_pose;

/* Traversed breadcrumb trail in mm (1 KB total) for guaranteed RETURN fallback. */
static int16_t trail_x[TRAIL_MAX],trail_y[TRAIL_MAX];
static unsigned trail_len;

static int coordinate(float v) { return CENTER+(int)floorf(v/CELL_M+0.50001f); }
static int cell(int x,int y) {
    return x>=0 && x<WIDTH && y>=0 && y<WIDTH ? x*WIDTH+y : -1;
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
    /* Mark the robot's traversed 3x3 footprint (15x15 cm) as observed free
       (unless confirmed wall) so fine 5 cm grids never leave '?' pinholes
       along the return corridor back to (0,0). */
    int cx=coordinate(x_m+offset_x), cy=coordinate(y_m+offset_y);
    for(int dx=-1;dx<=1;++dx) for(int dy=-1;dy<=1;++dy) {
        int ci=cell(cx+dx,cy+dy);
        if(ci>=0 && !occupied(ci)) map_set_state(&planner_map,ci,1);
    }
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
    waiting_for_route=false; route_ready=false; route_is_trail_fallback=false; need_leg=true;
    plan_stamp=W4Port_Millis()-300U; last_plan_scan=UINT32_MAX;
}
/* Called by the control task when the odometry origin changes. */
void W4Maze_Reset(void) {
    ++epoch; active=0; fresh=false; route_ready=false; route_is_trail_fallback=false; aim_valid=false;
    immediate_blocked=false; returning_mode=false; blocked_streak=0;
}

static void scan_clear(Scan *s,uint32_t ep) {
    memset(s->hit,0,sizeof s->hit);
    for(unsigned i=0;i<BEAMS;++i) s->ray[i].ox_mm=RAY_EMPTY;
    s->count=0; s->stamp=0; s->epoch=ep;
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
        scan_clear(&accumulating,epoch);
        scan_start=now;
        synchronized=true;
    }
    if(!synchronized || !quality || mm<50 || angle<0 || angle>=360) return;
    /* Bench telemetry showed a weak, repeating return near 0.33 m.
       Drop it before choosing a beam and do not clear space along it. */
    if(quality<=4 && mm>=300.0f && mm<=390.0f) return;
    unsigned bin=(unsigned)(angle*((float)BEAMS/360.0f));
    if(bin>=BEAMS) bin=BEAMS-1U;
    Ray *r=&accumulating.ray[bin];
    if(ray_valid(r)) return;
    W4Pose p;
    if(!W4Pose_GetSnapshot(&p)||!p.valid||now-p.stamp_ms>200) return;
    float a=p.theta_rad+W4_C1_ANGLE_SIGN*angle*(W4_PI/180.0f);
    float distance=fminf(mm*0.001f-W4_LIDAR_RANGE_OFFSET_M,6.0f);
    if(distance<0.05f) return;
    float c=cosf(p.theta_rad),s=sinf(p.theta_rad);
    float ox=p.x_m+c*W4_C1_FORWARD_OFFSET_M-s*W4_C1_LEFT_OFFSET_M;
    float oy=p.y_m+s*W4_C1_FORWARD_OFFSET_M+c*W4_C1_LEFT_OFFSET_M;
    float x=ox+distance*cosf(a), y=oy+distance*sinf(a);
    if(fabsf(ox)>30.0f || fabsf(oy)>30.0f || fabsf(x)>30.0f || fabsf(y)>30.0f) return;
    r->ox_mm=(int16_t)lroundf(ox*1000.0f);
    r->oy_mm=(int16_t)lroundf(oy*1000.0f);
    r->x_mm=(int16_t)lroundf(x*1000.0f);
    r->y_mm=(int16_t)lroundf(y*1000.0f);
    if(mm<=6000) ray_set_hit(&accumulating,bin);
    ++accumulating.count;
}

/* Normal-projected (point-to-line) translation ICP with Tikhonov damping,
   ZUPT, and turn guard. Damping prevents corridor-axis runaway drift. */
static void register_scan(const W4Pose *p) {
    if(!reference_count) {
        for(unsigned j=0;j<BEAMS;++j) if(ray_valid(&working.ray[j]) && ray_hit(&working,j)) {
            PointMm *r=&reference[reference_count++];
            r->x_mm=(int16_t)lroundf(working.ray[j].x_mm+offset_x*1000.0f);
            r->y_mm=(int16_t)lroundf(working.ray[j].y_mm+offset_y*1000.0f);
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
    unsigned k_step=reference_count>120U?2U:1U;
    for(unsigned iteration=0;iteration<3 && reference_count>=20;++iteration) {
        float a00=0,a01=0,a11=0,b0=0,b1=0;
        unsigned pairs=0;
        for(unsigned j=0;j<BEAMS;++j) {
            Ray *r=&working.ray[j];
            if(!ray_valid(r) || !ray_hit(&working,j)) continue;
            float x=r->x_mm*0.001f+tx, y=r->y_mm*0.001f+ty, best=0.07f*0.07f;
            int best_k=-1;
            for(unsigned k=0;k<reference_count;k+=k_step) {
                float rx=reference[k].x_mm*0.001f, ry=reference[k].y_mm*0.001f;
                float ex=rx-x, ey=ry-y, d=ex*ex+ey*ey;
                if(d<best) { best=d; best_k=(int)k; }
            }
            if(best_k<0) continue;
            unsigned k0=(best_k>0)?(unsigned)(best_k-1):(unsigned)best_k;
            unsigned k1=((unsigned)(best_k+1)<reference_count)?(unsigned)(best_k+1):(unsigned)best_k;
            float wx=(reference[k1].x_mm-reference[k0].x_mm)*0.001f;
            float wy=(reference[k1].y_mm-reference[k0].y_mm)*0.001f;
            float wlen=hypotf(wx,wy);
            if(k0==k1 || wlen<0.02f || wlen>0.35f) continue;
            float nx=-wy/wlen, ny=wx/wlen;
            float err=(reference[best_k].x_mm*0.001f-x)*nx + (reference[best_k].y_mm*0.001f-y)*ny;
            a00+=nx*nx; a01+=nx*ny; a11+=ny*ny;
            b0+=nx*err; b1+=ny*err;
            ++pairs;
        }
        /* Tikhonov damping suppresses corridor-axis runaway when only parallel
           walls are visible (one small eigenvalue of the 2x2 normal matrix). */
        if(pairs<16 || (a00+a11)<12.0f) break;
        const float damp=8.0f;
        float r00=a00+damp, r11=a11+damp;
        float rdet=r00*r11-a01*a01;
        if(rdet<1.0f) break;
        float dtx=(r11*b0-a01*b1)/rdet;
        float dty=(r00*b1-a01*b0)/rdet;
        tx+=0.5f*dtx; ty+=0.5f*dty;
    }
    if(hypotf(tx-offset_x,ty-offset_y)<=0.020f) {
        offset_x=fmaxf(-0.03f,fminf(0.03f,tx));
        offset_y=fmaxf(-0.03f,fminf(0.03f,ty));
    }
    if(hypotf(p->x_m-ref_x,p->y_m-ref_y)>0.24f ||
       fabsf(W4Wrap(p->theta_rad-ref_heading))>0.45f) {
        reference_count=0;
        for(unsigned j=0;j<BEAMS;++j) if(ray_valid(&working.ray[j]) && ray_hit(&working,j)) {
            PointMm *r=&reference[reference_count++];
            r->x_mm=(int16_t)lroundf(working.ray[j].x_mm+offset_x*1000.0f);
            r->y_mm=(int16_t)lroundf(working.ray[j].y_mm+offset_y*1000.0f);
        }
        ref_x=p->x_m; ref_y=p->y_m; ref_heading=p->theta_rad;
    }
}

static void trace(const Scan *s,unsigned bin) {
    const Ray *r=&s->ray[bin];
    if(!ray_valid(r) || (r->ox_mm==r->x_mm && r->oy_mm==r->y_mm)) return;
    float rox=r->ox_mm*0.001f, roy=r->oy_mm*0.001f;
    float rx_end=r->x_mm*0.001f, ry_end=r->y_mm*0.001f;
    bool hit=ray_hit(s,bin);
    int x=coordinate(rox+offset_x),y=coordinate(roy+offset_y);
    int ex=coordinate(rx_end+offset_x),ey=coordinate(ry_end+offset_y);
    int span_sq=(ex-x)*(ex-x)+(ey-y)*(ey-y);
    bool near_ray=!hit || span_sq<=(int)(1.5f/CELL_M)*(int)(1.5f/CELL_M);
    float vx=rx_end-rox,vy=ry_end-roy,ray_sq=vx*vx+vy*vy;
    int dx=abs(ex-x),dy=-abs(ey-y),sx=x<ex?1:-1,sy=y<ey?1:-1,err=dx+dy;
    for(;;) {
        int i=cell(x,y);
        if(i<0) break;
        if(x==ex && y==ey) break;
        if(map_state(&planner_map,i)==0) map_set_state(&planner_map,i,1);
        bool clear=near_ray || abs(x-ex)>1 || abs(y-ey)>1;
        if(!clear) {
            /* Clear an adjacent foreground ghost layer when the ray crosses
               near its centre (|cross| < 0.40 cell) and continues >=0.70 cell
               beyond the centre. Grazing corners keep protection; hits win. */
            float rx=rx_end+offset_x-(x-CENTER)*CELL_M;
            float ry=ry_end+offset_y-(y-CENTER)*CELL_M;
            float along=rx*vx+ry*vy,cross=rx*vy-ry*vx;
            float unit=CELL_M*CELL_M*ray_sq;
            clear=along>0 && along*along>0.49f*unit && cross*cross<0.16f*unit;
        }
        if(clear) scan_mark(i,1);
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
    char line[288],cells[WIDTH+1];
    for(unsigned x=0;x<WIDTH;++x) {
        int i=cell((int)x,(int)selected);
        cells[x]=occupied(i)?'#':observed_free(i)?'.':'?';
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
            Point2D sample={a.x+f*(b.x-a.x)+planner_origin,
                            a.y+f*(b.y-a.y)+planner_origin};
            float end_dist=hypotf(sample.x-planner_origin-(route[route_len-1].x+offset_x),
                                  sample.y-planner_origin-(route[route_len-1].y+offset_y));
            float clearance=end_dist<0.10f?PHYSICAL_RADIUS:PATH_CLEARANCE_RADIUS;
            if(index<0 || check_collision(&planner_map,sample,clearance,false)) {
                if(dist<0.32f) *near_block=true;
                return true;
            }
            if(returning_mode && !route_is_trail_fallback && !observed_free(index)) return true;
        }
        checked+=length; a=b;
        if(checked>=check_horizon) break;
    }
    return false;
}
/* Evidence for the next real run: distinguish occupied goal from clearance
   and retain the actual ICP-corrected endpoint and nearest blocking wall. */
static void report_goal_block(const W4NavSnapshot *n,uint32_t now) {
    float gx=n->target_x+offset_x,gy=n->target_y+offset_y;
    int cx=coordinate(gx),cy=coordinate(gy),bx=0,by=0;
    float nearest=1e9f;
    for(int x=cx-5;x<=cx+5;++x) for(int y=cy-5;y<=cy+5;++y) {
        int i=cell(x,y); if(i<0 || !occupied(i)) continue;
        float dx=(x-CENTER)*CELL_M-gx,dy=(y-CENTER)*CELL_M-gy;
        float d=dx*dx+dy*dy;
        if(d<nearest) { nearest=d; bx=x; by=y; }
    }
    int gi=cell(cx,cy);
    char line[160];
    int len=snprintf(line,sizeof line,"D,%lu,GOAL_CHECK,%lu,%ld,%ld,%ld,%ld,%ld,%u\n",
        (unsigned long)now,(unsigned long)n->cmd_id,lroundf(gx*1000),lroundf(gy*1000),
        lroundf((bx-CENTER)*CELL_M*1000),lroundf((by-CENTER)*CELL_M*1000),
        nearest<1e8f?lroundf(sqrtf(nearest)*1000):-1L,gi>=0 && occupied(gi)?1U:0U);
    if(len>0 && len<(int)sizeof line) (void)W4Telemetry_SubmitLine(line,(uint16_t)len);
}
void W4Maze_Poll(const W4NavSnapshot *n) {
    static uint32_t map_epoch;
    static uint32_t last_path_tx;
    uint32_t version=epoch;
    if(map_epoch!=version) {
        memset(grid,0,sizeof grid);
        memset(changed_rows,0,sizeof changed_rows);
        offset_x=offset_y=0; reference_count=0; have_prev_scan_pose=false; map_epoch=version;
        previous_len=0; route_ready=false; route_is_trail_fallback=false; aim_valid=false;
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
        memset(scan_flags,0,(CELLS+3)/4);
        for(unsigned i=0;i<BEAMS;++i) trace(&working,i);
        /* Per-revolution packed occupancy evidence with pass-through ghost erosion:
           - 3 consecutive hit scans confirm '#' (saturating at 6);
           - 3 pass-through scans (with 0 hits in that cell) erode drifted '#'
             layers back to '.' without resetting progress if a ray falls between
             beams; grid pitch is 5 cm with 1.5 deg angular binning;
           - occluded walls (neither hit nor passed through) stay '#' forever. */
        for(unsigned j=0;j<BEAMS;++j) if(ray_valid(&working.ray[j]) && ray_hit(&working,j)) {
            const Ray *ray=&working.ray[j];
            int i=index_at(ray->x_mm*0.001f+offset_x,ray->y_mm*0.001f+offset_y);
            if(i<0) continue;
            scan_mark(i,2);
        }
        for(int i=0;i<CELLS;++i) {
            unsigned state=map_state(&planner_map,i),next=state;
            unsigned flags=(scan_flags[i/4]>>((i%4)*2))&3U;
            if(flags&2U) next=state>=4?6:state==2?3:state==3?6:2;
            else if(state>=4) {
                /* Erode on clean pass-through; keep current score unchanged when
                   occluded or missed between beams so 5 cm ghosts can clear. */
                next=(flags&1U)?(state==4?1:state-1):state;
            } else if(state==2 || state==3) next=(flags&1U)?1:0;
            if((state>=4)!=(next>=4)) {
                unsigned row=(unsigned)i%WIDTH;
                changed_rows[row/8]|=(uint8_t)(1U<<(row%8));
            }
            map_set_state(&planner_map,i,(uint8_t)next);
        }
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
    uint32_t min_interval=(returning_mode && route_ready && !immediate_blocked)?1500U:300U;
    if(now-plan_stamp<min_interval) return;
    if(waiting_for_route && fresh && last_plan_scan==last_scan) return;
    plan_stamp=now; last_plan_scan=last_scan;
    /* Match NoeticMaze: never fail with FAULT 9 when a scan is delayed. Plan on
       the latest map (or breadcrumb fallback for RETURN) and continuously refine. */
    bool ok=false, used_trail=false;
    if(returning_mode) {
        ok=plan(n->pose.x_m+offset_x,n->pose.y_m+offset_y,offset_x,offset_y,true);
        if(!ok) ok=plan(n->pose.x_m+offset_x,n->pose.y_m+offset_y,offset_x,offset_y,false);
        if(!ok) { ok=build_trail_return_route(n->pose.x_m,n->pose.y_m); used_trail=ok; }
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
            if(plan_failure && !strcmp(plan_failure,"GOAL_BLOCKED")) report_goal_block(n,now);
        }
        return;
    }
    bool resumed=waiting_for_route;
    waiting_for_route=false;
    route_is_trail_fallback=used_trail;
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
