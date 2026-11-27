#include "w4_nav.h"
#ifdef W4_MAZE
#include "w4_maze.h"
#endif
#include "w4_rtos.h"
#include "w4_runtime.h"
#include <math.h>
/* Path task is sole producer AND sole queue-reset owner. Stream bounded points,
   not an unbounded allocation. Coordinate routes use the current pose. */
static W4Waypoint pending;
static float vx[3],vy[3],sx,sy;
static unsigned vertex,vertices,part,parts;
static bool producing,have_pending;
static uint32_t path_id,wait_since;
static uint16_t index_wp;
static W4Command prefetched;
static bool have_command;
#ifndef W4_DUAL_ENCODER
static bool near(float a,float b) { return fabsf(a-b)<0.001f; }
#endif
static void reject(uint32_t id,const char *reason) {
    uint32_t k=W4Port_Lock();
    if(w4_nav.cmd_id==id) w4_nav.state=W4_IDLE;
    W4Port_Unlock(k); W4Ack(id,"REJECTED",reason);
}
static bool start(const W4Command *c) {
    W4Pose p; W4ImuSample imu;
    if(!W4HardwareReady()) { reject(c->cmd_id,"HARDWARE_NOT_READY"); return false; }
    bool have_pose=W4Pose_GetSnapshot(&p);
    bool have_imu=W4Imu_GetSnapshot(&imu);
    /* Higher-priority producers may publish between these snapshot copies.
       Read the age reference afterwards to avoid unsigned future-age wrap. */
    uint32_t now=W4Port_Millis();
#ifdef W4_MAZE
    if(c->type==W4_CMD_MAZE || c->type==W4_CMD_RETURN) {
        if(!have_pose) { reject(c->cmd_id,"POSE_NOT_READY"); return false; }
        W4Maze_Begin(c->cmd_id,c->type==W4_CMD_RETURN);
        return false; /* maze service plans and hot-swaps full routes */
    }
#endif
    if(!have_pose||now-p.stamp_ms>50||
       !have_imu||!imu.valid||!imu.calibrated||now-imu.stamp_ms>50) {
        reject(c->cmd_id,"POSE_NOT_READY"); return false;
    }
#ifdef W4_DUAL_ENCODER
    if(c->x_m==0.0f&&c->y_m>=0.0f) {
        pending=(W4Waypoint){.cmd_id=c->cmd_id,.index=W4_DISTANCE_WAYPOINT,
                             .last=1,.y_m=c->y_m};
        path_id=c->cmd_id;
        producing=have_pending=true;
        wait_since=now;
        W4Ack(path_id,"OK","PATH_ACCEPTED");
        return true;
    }
    sx=p.x_m;
    sy=p.y_m;
    vertices=0;
    if(fabsf(c->y_m-sy)>=0.001f&&fabsf(c->x_m-sx)>=0.001f) {
        vx[vertices]=sx;
        vy[vertices++]=c->y_m;
    }
    vx[vertices]=c->x_m;
    vy[vertices++]=c->y_m;
#else
    if(hypotf(p.x_m,p.y_m)>0.01f) { reject(c->cmd_id,"RESET_AT_START"); return false; }
    sx=sy=0.0f;
    vertices=0;
    if(near(c->x_m,0)&&near(c->y_m,1.4f)) { vx[0]=0; vy[0]=1.4f; vertices=1; }
    else if((near(c->x_m,-0.7f)&&near(c->y_m,0.7f)) ||
            (near(c->x_m,0.7f)&&near(c->y_m,0.7f))) {
        vx[0]=0; vy[0]=0.7f; vx[1]=c->x_m; vy[1]=0.7f; vertices=2;
    } else if(near(c->x_m,0.7f)&&near(c->y_m,-0.7f)) {
        /* Text target. Orthogonal negative-Y corridor is an assumption that
           must be confirmed on the physical maze before motion is unlocked. */
        vx[0]=0; vy[0]=-0.7f; vx[1]=0.7f; vy[1]=-0.7f; vertices=2;
    } else if(near(c->x_m,0)&&near(c->y_m,0)) { vx[0]=0; vy[0]=0; vertices=1; }
    else { reject(c->cmd_id,"ROUTE_UNCONFIRMED"); return false; }
#endif
    vertex=part=parts=0; path_id=c->cmd_id; index_wp=0;
    producing=true; have_pending=false; wait_since=now;
    W4Ack(path_id,"OK","PATH_ACCEPTED"); return true;
}
static void poll(void) {
    uint32_t k=W4Port_Lock();
    bool stopping=w4_stop_pending;
    bool consumed=stopping&&w4_control_stop_epoch==w4_stop_epoch;
    uint32_t epoch=w4_stop_epoch;
    W4Port_Unlock(k);
    if(stopping) {
        producing=have_pending=have_command=false;
        if(consumed) {
            (void)osMessageQueueReset(Q_Path);
            (void)osMessageQueueReset(Q_Cmd);
            k=W4Port_Lock(); if(epoch==w4_stop_epoch) w4_stop_pending=false; W4Port_Unlock(k);
        }
        return;
    }
    W4NavSnapshot n; W4Nav_GetSnapshot(&n);
#ifdef W4_MAZE
    W4Maze_Poll(&n);
#endif
    if(producing&&(n.cmd_id!=path_id||n.state==W4_FAULT||n.state==W4_STOPPED)) {
        producing=have_pending=false; (void)osMessageQueueReset(Q_Path);
    }
    if(!producing) {
        W4Command c;
        if(have_command) { c=prefetched; have_command=false; }
        else if(osMessageQueueGet(Q_Cmd,&c,0,0)!=osOK) return;
        /* RX may reserve PLANNING while this task was blocked in Get. */
        W4Nav_GetSnapshot(&n);
        k=W4Port_Lock(); stopping=w4_stop_pending; W4Port_Unlock(k);
        if(stopping) return;
        if((c.type!=W4_CMD_GOTO&&c.type!=W4_CMD_MAZE&&c.type!=W4_CMD_RETURN)||c.cmd_id!=n.cmd_id||n.state!=W4_PLANNING) return;
        (void)osMessageQueueReset(Q_Path);
        if(!start(&c)) return;
    }
    /* At most one pending item per poll. Retry same value on backpressure. */
    if(!have_pending) {
        if(!parts) {
            float d=hypotf(vx[vertex]-sx,vy[vertex]-sy);
            parts=(unsigned)ceilf(d/w4_config.step_m); if(!parts) parts=1; part=0;
        }
        float f=(float)(part+1)/(float)parts;
        bool leg_end=part+1==parts;
        uint16_t index=index_wp;
        float dx=vx[vertex]-sx,dy=vy[vertex]-sy;
        if(fabsf(dx)>=0.001f||fabsf(dy)>=0.001f) {
            bool along_y=fabsf(dy)>fabsf(dx);
            index|=along_y?W4_WAYPOINT_AXIS_Y:W4_WAYPOINT_AXIS_X;
            if((along_y?dy:dx)<0.0f) index|=W4_WAYPOINT_NEGATIVE;
        }
        if(vertex+1<vertices) {
            index|=W4_WAYPOINT_INCOMING_LEG;
            if(leg_end) index|=W4_CORNER_WAYPOINT;
        }
        pending=(W4Waypoint){path_id,index,(uint8_t)(vertex+1==vertices&&leg_end),
            sx+(vx[vertex]-sx)*f,sy+(vy[vertex]-sy)*f};
        have_pending=true;
    }
    if(osMessageQueuePut(Q_Path,&pending,0,0)!=osOK) {
        ++w4_stats.path_wait;
        if(W4Port_Millis()-wait_since>5000) {
            W4Nav_RequestStop(); W4Ack(path_id,"REJECTED","PATH_BACKPRESSURE");
        }
        return;
    }
    wait_since=W4Port_Millis(); have_pending=false;
    if(pending.index==W4_DISTANCE_WAYPOINT) { producing=false; return; }
    ++index_wp;
    if(++part==parts) { sx=vx[vertex]; sy=vy[vertex]; ++vertex; parts=0; }
    if(vertex==vertices) producing=false;
}
void W4Path_Poll(void) {
    W4Runtime_Begin(W4_TASK_PATH,osKernelGetTickCount());
    poll();
    W4Runtime_End(W4_TASK_PATH,0);
}
void W4PathTask(void *argument) {
    (void)argument;
    for(;;) {
        /* bounded queue wait keeps STOP responsive even if its queue put failed */
        /* Blocking command wait is outside the useful-work measurement. */
        if(!producing&&!w4_stop_pending)
            have_command=osMessageQueueGet(Q_Cmd,&prefetched,0,10)==osOK;
        W4Path_Poll();
        /* Backpressure yields to the consumer; no full-queue spin loop. */
        (void)osDelay(1); /* also yield while waiting for control to consume STOP */
    }
}
