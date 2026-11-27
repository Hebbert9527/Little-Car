#include "w4_nav.h"
#include "w4_rtos.h"
#include "w4_runtime.h"
#include <math.h>
#include <string.h>
static W4Waypoint wp;
static bool have_wp,have_time;
static W4Pose reset_pose;
static float reset_offset;
static uint32_t last_ms,missing_since,arrival_since,command_since;
static bool arriving,missing;
static uint32_t controller_id;
static float segment_x,segment_y,final_heading;
static W4Pid heading_pid;
static float stall_time[2];
bool W4Nav_ResetPose(float x,float y,float theta) {
    if(!isfinite(x)||!isfinite(y)||!isfinite(theta)||fabsf(x)>2||fabsf(y)>2) return false;
    W4ImuSample imu;
    if(!W4Imu_GetSnapshot(&imu)||!imu.valid||!imu.calibrated||
       W4Port_Millis()-imu.stamp_ms>50||!isfinite(imu.heading_rad)) return false;
    float offset=W4Wrap(theta-imu.heading_rad*w4_config.imu_sign);
    uint32_t k=W4Port_Lock();
    bool ok=!w4_stop_pending&&!w4_reset_pending&&w4_nav.state!=W4_PLANNING&&w4_nav.state!=W4_TRACKING;
    if(ok) { reset_pose=(W4Pose){0,x,y,W4Wrap(theta),1}; reset_offset=offset; w4_reset_pending=true; }
    W4Port_Unlock(k); return ok;
}
static void state(W4State s,uint32_t fault) {
    uint32_t k=W4Port_Lock(); w4_nav.state=s; w4_nav.fault=fault; W4Port_Unlock(k);
}
static void stop_control(void) {
    have_wp=false; arriving=missing=false;
    W4Pid_Reset(&heading_pid); stall_time[0]=stall_time[1]=0; W4Motor_Stop();
}
static void fault(uint32_t code) {
    stop_control();
    uint32_t k=W4Port_Lock();
    w4_nav.pose.valid=0;
    w4_nav.fault=code;
    w4_nav.state=W4_FAULT;
    W4Port_Unlock(k);
}
static void control_step(uint32_t now) {
    uint32_t begin=W4Port_Micros(),k=W4Port_Lock();
    bool stopping=w4_stop_pending;
    if(stopping) {
        w4_control_stop_epoch=w4_stop_epoch; w4_nav.state=W4_STOPPED;
        w4_nav.cmd_id=0; w4_reset_pending=false;
    }
    bool resetting=w4_reset_pending&&!stopping;
    if(resetting) {
        w4_nav.pose=reset_pose; w4_nav.pose.stamp_ms=now;
        w4_config.imu_offset_rad=reset_offset;
        w4_nav.state=W4_STOPPED; w4_nav.fault=0; w4_nav.cmd_id=0;
        w4_nav.target_x=reset_pose.x_m; w4_nav.target_y=reset_pose.y_m;
        w4_nav.wp_index=0; w4_reset_pending=false; have_time=false;
    }
    W4Port_Unlock(k);
    if(stopping) { stop_control(); last_ms=now; have_time=true; return; }
    W4EncoderDelta e; W4Encoder_TakeDelta(&e);
    /* Encoder acquisition may straddle the HAL millisecond tick. Use a time
       captured after the snapshot for freshness and this cycle's dt. */
    now=W4Port_Millis();
    /* Drain pre-reset edge counts, without integrating them in the new frame. */
    if(resetting) { stop_control(); last_ms=now; have_time=true; return; }
    float dt=have_time?(now-last_ms)*0.001f:0.01f; last_ms=now; have_time=true;
    float vl=0,vr=0;
    bool sensors=W4Pose_Update(now,&e,dt,&vl,&vr);
    w4_stats.pose_us=W4Port_Micros()-begin;
    W4NavSnapshot n; W4Nav_GetSnapshot(&n);
    bool active=n.state==W4_PLANNING||n.state==W4_TRACKING;
    if(!active) { stop_control(); return; }
    if(controller_id!=n.cmd_id) {
        stop_control(); controller_id=n.cmd_id; command_since=now;
        segment_x=n.pose.x_m; segment_y=n.pose.y_m; final_heading=n.pose.theta_rad;
    }
    if(now-command_since>120000) { fault(2); return; }
    if(!sensors && n.state==W4_TRACKING) { fault(1); return; }
    if(!have_wp) {
        W4Waypoint next;
        /* bounded stale drain: avoid monopolising a high-priority task */
        for(unsigned j=0;j<64&&osMessageQueueGet(Q_Path,&next,0,0)==osOK;++j) {
            if(next.cmd_id!=n.cmd_id) { ++w4_stats.stale_wp; continue; }
            /* A waypoint for this command proves the planner accepted it.
               Until then it may still reject missing calibration or pose. */
            if(!sensors) { fault(1); return; }
            if(!isfinite(next.x_m)||!isfinite(next.y_m)) { fault(3); return; }
            wp=next; have_wp=true; missing=false;
            if(hypotf(wp.x_m-segment_x,wp.y_m-segment_y)>0.0001f)
                final_heading=atan2f(wp.y_m-segment_y,wp.x_m-segment_x);
            k=W4Port_Lock(); w4_nav.wp_index=wp.index; w4_nav.state=W4_TRACKING; W4Port_Unlock(k);
            break;
        }
        if(!have_wp) {
            W4Motor_Stop();
            if(!missing) { missing=true; missing_since=now; }
            if(now-missing_since>1000) fault(4);
            return; /* empty queue is never arrival */
        }
    }
    float dx=wp.x_m-n.pose.x_m,dy=wp.y_m-n.pose.y_m,d=hypotf(dx,dy);
    /* Tight intermediate tolerance keeps a bend on its corridor centreline. */
    float tol=wp.last?w4_config.position_tol:0.008f;
    if(d<tol&&!wp.last) {
        segment_x=wp.x_m; segment_y=wp.y_m; have_wp=false;
        W4Motor_Stop(); W4Pid_Reset(&heading_pid); return;
    }
    float desired=(wp.last&&d<tol)?final_heading:atan2f(dy,dx);
    float error=W4Wrap(desired-n.pose.theta_rad);
    if(wp.last&&d<tol&&fabsf(W4Wrap(final_heading-n.pose.theta_rad))<w4_config.heading_tol) {
        W4Motor_Stop();
        if(!arriving) { arriving=true; arrival_since=now; }
        if(now-arrival_since>=200) { stop_control(); state(W4_ARRIVED,0); }
        return;
    }
    arriving=false;
    float v=(d<tol||fabsf(error)>0.35f)?0:fminf(w4_config.max_v,1.5f*d)*fmaxf(0,cosf(error));
    if(heading_pid.have_previous)
        heading_pid.previous=error-W4Wrap(error-heading_pid.previous);
    float w=W4Pid_Update(&heading_pid,error,dt,w4_config.heading_kp,w4_config.heading_ki,
                        w4_config.heading_kd,w4_config.max_w);
    float l=v-w*w4_config.track_m*0.5f,r=v+w*w4_config.track_m*0.5f;
    float peak=fmaxf(fabsf(l),fabsf(r));
    if(peak>w4_config.max_wheel) { l*=w4_config.max_wheel/peak; r*=w4_config.max_wheel/peak; }
    float req[2]={l,r},meas[2]={vl,vr};
    for(unsigned j=0;j<2;++j) {
#ifdef W4_SINGLE_ENCODER
        if(j==0) continue; /* Left speed is inferred, never independent stall evidence. */
#endif
        stall_time[j]=(fabsf(req[j])>0.03f&&fabsf(meas[j])<0.003f)?stall_time[j]+dt:0;
        if(stall_time[j]>1.0f) { fault(5); return; }
    }
    w4_stats.tracking_us=W4Port_Micros()-begin-w4_stats.pose_us;
    uint32_t motor_begin=W4Port_Micros();
    /* STOP may arrive during the computation. Commit PWM atomically with the
       final latch check, so a completed STOP cannot be followed by new PWM. */
    W4Motor_SetWheelSpeeds(l,r); W4Motor_Update(vl,vr,dt,now);
    w4_stats.motor_us=W4Port_Micros()-motor_begin;
}
void W4Control_Step(uint32_t now) {
    uint32_t begin=W4Port_Micros();
    w4_stats.pose_us=w4_stats.tracking_us=w4_stats.motor_us=0;
    control_step(now);
    w4_stats.cycle_us=W4Port_Micros()-begin;
    if(w4_stats.cycle_us>w4_stats.max_cycle_us) w4_stats.max_cycle_us=w4_stats.cycle_us;
}
void W4ControlTask(void *argument) {
    (void)argument;
    uint32_t period=osKernelGetTickFreq()/100,deadline=osKernelGetTickCount();
    if(period==0||osKernelGetTickFreq()!=1000) { W4Motor_Stop(); return; }
    for(;;) {
        W4Runtime_Begin(W4_TASK_CONTROL,deadline);
        W4Control_Step(W4Port_Millis());
        W4Runtime_End(W4_TASK_CONTROL,period);
        deadline+=period;
        uint32_t now=osKernelGetTickCount();
        if((int32_t)(deadline-now)<=0) {
            ++w4_stats.deadline_miss;
            W4NavSnapshot n; W4Nav_GetSnapshot(&n);
            if(n.state==W4_PLANNING||n.state==W4_TRACKING) fault(6); else W4Motor_Stop();
            deadline=now+period; /* discard expired releases, no catch-up spin */
        }
        (void)osDelayUntil(deadline);
    }
}
