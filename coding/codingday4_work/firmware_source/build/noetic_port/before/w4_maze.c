#ifdef W4_MAZE
#include "w4_maze.h"
#include "w4_rtos.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

enum { WIDTH=49, CENTER=24, CELLS=WIDTH*WIDTH, BEAMS=120, CLEARANCE=2 };
enum { NEAR_HIT_SCANS=3, HIT_THIS_SCAN=0x80 };
#define CELL_M 0.10f
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
static int8_t grid[CELLS];
static uint8_t hit_scans[CELLS];
static float offset_x,offset_y,leg_x,leg_y;
static int16_t first_leg[CELLS],cost[CELLS];
static uint8_t closed[CELLS],wall[CELLS];
static Ray reference[BEAMS];
static unsigned reference_count;
static float ref_x,ref_y,ref_heading;

static int coordinate(float v) { return CENTER+(int)floorf(v/CELL_M+0.5f); }
static int cell(int x,int y) {
    return x>=0 && x<WIDTH && y>=0 && y<WIDTH ? y*WIDTH+x : -1;
}
static int index_at(float x,float y) { return cell(coordinate(x),coordinate(y)); }
bool W4Maze_Active(uint32_t id) { return id && active==id; }
bool W4Maze_Fresh(uint32_t now) { return fresh && scan_epoch==epoch && now-last_scan<=450U; }
bool W4Maze_LegBlocked(void) { return blocked; }
bool W4Maze_FinalLeg(void) { return final_leg; }
void W4Maze_NextLeg(void) { blocked=false; need_leg=true; }
void W4Maze_Begin(uint32_t id) { active=id; ack_pending=true; W4Maze_NextLeg(); }
/* Called by the control task when the odometry origin changes. */
void W4Maze_Reset(void) { ++epoch; active=0; fresh=false; }

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
        if(grid[i]>-8) --grid[i];
        int twice=2*err;
        if(twice>=dy) { err+=dy; x+=sx; }
        if(twice<=dx) { err+=dx; y+=sy; }
    }
}
/* Publish a rolling snapshot. A lost row is refreshed on the next sweep. */
static void publish_row(uint32_t version) {
    static unsigned row;
    static uint32_t sent;
    uint32_t now=W4Port_Millis();
    if(now-sent<100U) return;
    char line[100],cells[WIDTH+1];
    for(unsigned x=0;x<WIDTH;++x) {
        int value=grid[row*WIDTH+x];
        cells[x]=value>0?'#':value<0 && !hit_scans[row*WIDTH+x]?'.':'?';
    }
    cells[WIDTH]=0;
    int len=snprintf(line,sizeof line,"M,%lu,%u,%ld,%ld,%s\n",
        (unsigned long)version,row,lroundf(offset_x*1000),lroundf(offset_y*1000),cells);
    if(len>0 && len<(int)sizeof line && W4Telemetry_SubmitLine(line,(uint16_t)len))
        row=(row+1)%WIDTH;
    sent=now;
}
static void inflate(void) {
    memset(wall,0,sizeof wall);
    for(int i=0;i<CELLS;++i) if(grid[i]>0 || hit_scans[i]) {
        for(int dy=-CLEARANCE;dy<=CLEARANCE;++dy)
            for(int dx=-CLEARANCE;dx<=CLEARANCE;++dx) {
                if(dx*dx+dy*dy>CLEARANCE*CLEARANCE) continue;
                int n=cell(i%WIDTH+dx,i/WIDTH+dy);
                if(n>=0) wall[n]=1;
            }
    }
}
static bool free_cell(int i) {
    return i>=0 && !wall[i] && grid[i]<0;
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

static int manhattan(int a,int b) {
    return abs(a%WIDTH-b%WIDTH)+abs(a/WIDTH-b/WIDTH);
}
static float cell_center(int coordinate_index) {
    return (coordinate_index-CENTER)*CELL_M;
}

/* Seed A* only with executable, observed first legs. The remaining route
   can cross unknown cells at a penalty and is replanned after each stop. */
static bool plan(float x,float y,float gx,float gy,float *nx,float *ny,bool *last) {
    const float max_leg=0.46f;
    int start=index_at(x,y),goal=index_at(gx,gy);
    if(!free_cell(start) || goal<0 || wall[goal]) return false;
    if(hypotf(gx-x,gy-y)<=max_leg && clear_line(x,y,gx,gy)) {
        *nx=gx; *ny=gy; *last=true;
        return true;
    }

    if(start==goal) return false;

    memset(cost,0x7f,sizeof cost);
    memset(closed,0,sizeof closed);
    closed[start]=1;
    int radius=(int)ceilf(max_leg/CELL_M);
    for(int row=start/WIDTH-radius;row<=start/WIDTH+radius;++row) {
        for(int col=start%WIDTH-radius;col<=start%WIDTH+radius;++col) {
            int i=cell(col,row);
            if(i==start || !free_cell(i)) continue;
            float cx=cell_center(col),cy=cell_center(row);
            float distance=hypotf(cx-x,cy-y);
            if(distance<0.075f || distance>max_leg || !clear_line(x,y,cx,cy)) continue;
            cost[i]=manhattan(start,i);
            first_leg[i]=(int16_t)i;
        }
    }

    const int dx[]={0,0,-1,1},dy[]={1,-1,0,0};
    for(;;) {
        int best=-1,score=32767,first_distance=-1;
        for(int i=0;i<CELLS;++i) {
            if(closed[i] || cost[i]>=0x7f7f) continue;
            int f=cost[i]+manhattan(i,goal);
            int leg_distance=manhattan(start,first_leg[i]);
            if(f<score || (f==score && leg_distance>first_distance)) {
                best=i; score=f; first_distance=leg_distance;
            }
        }
        if(best<0) return false;
        if(best==goal) {
            int leg=first_leg[best];
            *nx=cell_center(leg%WIDTH);
            *ny=cell_center(leg/WIDTH);
            *last=false;
            return true;
        }
        closed[best]=1;
        for(int direction=0;direction<4;++direction) {
            int n=cell(best%WIDTH+dx[direction],best/WIDTH+dy[direction]);
            if(n<0 || wall[n] || closed[n]) continue;
            int next_cost=cost[best]+(grid[n]<0?1:5);
            if(next_cost<cost[n]) {
                cost[n]=next_cost;
                first_leg[n]=first_leg[best];
            }
        }
    }
}
void W4Maze_Poll(const W4NavSnapshot *n) {
    static uint32_t map_epoch;
    uint32_t version=epoch;
    if(map_epoch!=version) {
        memset(grid,0,sizeof grid); memset(wall,0,sizeof wall);
        memset(hit_scans,0,sizeof hit_scans);
        offset_x=offset_y=0; reference_count=0; map_epoch=version;
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
            if(!ray->near) hit_scans[i]=HIT_THIS_SCAN|NEAR_HIT_SCANS;
            else hit_scans[i]|=HIT_THIS_SCAN;
        }
        for(int i=0;i<CELLS;++i) {
            if(!(hit_scans[i]&HIT_THIS_SCAN)) { hit_scans[i]=0; continue; }
            unsigned count=(hit_scans[i]&~HIT_THIS_SCAN)+1U;
            hit_scans[i]=count>=NEAR_HIT_SCANS?NEAR_HIT_SCANS:count;
            if(hit_scans[i]==NEAR_HIT_SCANS && grid[i]<9)
                grid[i]=grid[i]<1?2:grid[i]+1;
        }
        inflate();
        scan_epoch=version; last_scan=working.stamp; fresh=true;
        if(W4Maze_Active(n->cmd_id) && n->state==W4_TRACKING &&
           !clear_line(n->pose.x_m+offset_x,n->pose.y_m+offset_y,
                       leg_x+offset_x,leg_y+offset_y)) blocked=true;
    }
    if(W4Maze_Fresh(W4Port_Millis())) publish_row(version);
    if(!W4Maze_Active(n->cmd_id)||n->state!=W4_PLANNING||!need_leg) return;
    float nx=0,ny=0; bool last=false;
    bool valid=W4Maze_Fresh(W4Port_Millis());
    bool ok=valid && plan(n->pose.x_m+offset_x,n->pose.y_m+offset_y,
                         n->target_x,n->target_y,&nx,&ny,&last);
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
    leg_x=nx-offset_x; leg_y=ny-offset_y;
    final_leg=last; blocked=false;
    W4Waypoint wp={.cmd_id=n->cmd_id,.last=1,.x_m=leg_x,.y_m=leg_y};
    W4Port_Unlock(key);
    if(osMessageQueuePut(Q_Path,&wp,0,0)==osOK) {
        need_leg=false;
        char line[96];
        int len=snprintf(line,sizeof line,"D,%lu,MAZE_LEG,%lu,%ld,%ld\n",
                         (unsigned long)W4Port_Millis(),(unsigned long)wp.cmd_id,
                         lroundf(wp.x_m*1000),lroundf(wp.y_m*1000));
        if(len>0 && len<(int)sizeof line) W4Telemetry_SubmitLine(line,(uint16_t)len);
        if(ack_pending) { W4Ack(n->cmd_id,"OK","MAZE_ACCEPTED"); ack_pending=false; }
    }
}
#endif
