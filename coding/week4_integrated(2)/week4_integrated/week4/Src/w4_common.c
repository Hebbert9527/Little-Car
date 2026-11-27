#include "w4_nav.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
W4Config w4_config={
    .imu_offset_rad=W4_PI/2, .imu_sign=1,
    .wheel_kp=1.0f,.wheel_ki=2.0f,.wheel_kd=0.0f,.wheel_ff=1.0f,
    .heading_kp=2.0f,.heading_ki=0.0f,.heading_kd=0.05f,
    .max_v=0.15f,.max_w=1.0f,.max_wheel=0.25f,.pwm_limit=0.4f,
    .position_tol=0.05f,.heading_tol=5.0f*W4_PI/180.0f,.step_m=0.05f
};
W4NavSnapshot w4_nav;
W4Stats w4_stats;
bool w4_stop_pending;
bool w4_reset_pending;
uint32_t w4_stop_epoch,w4_control_stop_epoch;
float W4Wrap(float a) {
    if(!isfinite(a)) return NAN;
    a=fmodf(a+W4_PI,2*W4_PI); if(a<0) a+=2*W4_PI; return a-W4_PI;
}
int32_t W4CountDelta(uint32_t a,uint32_t b) {
    uint32_t d=a-b;
    return d<=INT32_MAX ? (int32_t)d : -(int32_t)(UINT32_MAX-d)-1;
}
void W4Pid_Reset(W4Pid *p) { memset(p,0,sizeof(*p)); }
float W4Pid_Update(W4Pid *p,float e,float dt,float kp,float ki,float kd,float lim) {
    if(!isfinite(e)||!isfinite(dt)||dt<=0||lim<=0) { W4Pid_Reset(p); return 0; }
    float raw=p->have_previous?(e-p->previous)/dt:0;
    p->derivative+=dt/(0.04f+dt)*(raw-p->derivative);
    float next=p->integral+ki*e*dt;
    float u=kp*e+next+kd*p->derivative;
    if(fabsf(u)<=lim || (u>lim&&e<0) || (u< -lim&&e>0)) p->integral=next;
    u=kp*e+p->integral+kd*p->derivative;
    p->previous=e; p->have_previous=true;
    return fmaxf(-lim,fminf(lim,u));
}
bool W4HardwareReady(void) {
    W4Config *c=&w4_config;
    return c->verified&&c->encoders_verified&&c->imu_axes_verified&&W4Port_EncodersReady()
      &&isfinite(c->diameter_l_m)&&c->diameter_l_m>0&&c->diameter_l_m<1
      &&isfinite(c->diameter_r_m)&&c->diameter_r_m>0&&c->diameter_r_m<1
      &&isfinite(c->track_m)&&c->track_m>0&&c->track_m<1
      &&isfinite(c->cpr_l)&&c->cpr_l>0&&isfinite(c->cpr_r)&&c->cpr_r>0
      &&abs(c->encoder_sign_l)==1&&abs(c->encoder_sign_r)==1&&abs(c->imu_sign)==1
      &&isfinite(c->imu_offset_rad)&&c->max_v>0&&c->max_v<=1
      &&c->max_w>0&&c->max_w<=5&&c->max_wheel>0&&c->max_wheel<=1
      &&c->pwm_limit>0&&c->pwm_limit<=1&&c->step_m>=0.01f&&c->step_m<=0.1f
      &&c->position_tol>0&&c->position_tol<=0.05f&&c->heading_tol>0&&c->heading_tol<=0.1f
      &&isfinite(c->wheel_kp)&&isfinite(c->wheel_ki)&&isfinite(c->wheel_kd)
      &&isfinite(c->wheel_ff)&&isfinite(c->pwm_deadzone)&&c->pwm_deadzone>=0&&c->pwm_deadzone<c->pwm_limit
      &&isfinite(c->heading_kp)&&isfinite(c->heading_ki)&&isfinite(c->heading_kd);
}
void W4Nav_Init(void) {
    memset(&w4_nav,0,sizeof(w4_nav)); memset(&w4_stats,0,sizeof(w4_stats));
    w4_stop_pending=w4_reset_pending=false; w4_stop_epoch=w4_control_stop_epoch=0;
    w4_nav.pose=(W4Pose){W4Port_Millis(),0,0,W4_PI/2,1}; W4Motor_Stop();
}
void W4Nav_RequestStop(void) {
    uint32_t k=W4Port_Lock(); ++w4_stop_epoch; w4_stop_pending=true; W4Port_Unlock(k);
}
bool W4Nav_GetSnapshot(W4NavSnapshot *o) {
    if(!o) return false;
    uint32_t k=W4Port_Lock(); *o=w4_nav; W4Port_Unlock(k); return true;
}
void W4Nav_GetStats(W4Stats *o) { uint32_t k=W4Port_Lock(); *o=w4_stats; W4Port_Unlock(k); }
bool W4Pose_GetSnapshot(W4Pose *o) {
    if(!o) return false;
    uint32_t k=W4Port_Lock(); *o=w4_nav.pose; W4Port_Unlock(k); return o->valid!=0;
}
void W4Ack(uint32_t id,const char *s,const char *r) {
    char b[96]; int n=snprintf(b,sizeof b,"A,%lu,%s,%s\n",(unsigned long)id,s,r);
    if(n<=0||n>=(int)sizeof b||!W4Telemetry_SubmitLine(b,(uint16_t)n)) ++w4_stats.ack_lost;
}
uint16_t W4Nav_FormatTelemetry(char *out,uint16_t capacity,uint32_t now) {
    static uint32_t seq;
    static const char *names[]={"IDLE","PLANNING","TRACKING","ARRIVED","STOPPED","FAULT"};
    W4NavSnapshot n; W4Nav_GetSnapshot(&n);
    if(!out||capacity<160||n.state>W4_FAULT) return 0;
    int len=snprintf(out,capacity,"N,%lu,%lu,%lu,%s,%ld,%ld,%ld,%ld,%ld,%u,%lu\n",
        (unsigned long)seq++,(unsigned long)now,(unsigned long)n.cmd_id,names[n.state],
        (long)lroundf(n.pose.x_m*1000),(long)lroundf(n.pose.y_m*1000),
        (long)lroundf(n.pose.theta_rad*1000),(long)lroundf(n.target_x*1000),
        (long)lroundf(n.target_y*1000),(unsigned)n.wp_index,(unsigned long)n.fault);
    return len>0&&len<capacity?(uint16_t)len:0;
}
