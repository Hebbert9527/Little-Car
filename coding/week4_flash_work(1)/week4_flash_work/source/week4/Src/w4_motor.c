#include "w4_nav.h"
#include <math.h>
static float requested[2];
static W4Pid pid[2];
static int direction[2];
static uint32_t zero_since[2];
static bool zero_valid[2];
static float clamp(float x,float a) { return fmaxf(-a,fminf(a,x)); }
void W4Motor_SetWheelSpeeds(float l,float r) {
    if(!isfinite(l)||!isfinite(r)) { requested[0]=requested[1]=0; return; }
    requested[0]=clamp(l,w4_config.max_wheel); requested[1]=clamp(r,w4_config.max_wheel);
}
void W4Motor_Stop(void) {
    uint32_t now=W4Port_Millis();
    requested[0]=requested[1]=0;
    for(unsigned j=0;j<2;++j) {
        W4Pid_Reset(&pid[j]);
        if(!zero_valid[j]) { zero_since[j]=now; zero_valid[j]=true; }
    }
    W4Port_WritePwm(0,0);
}
void W4Motor_Update(float vl,float vr,float dt,uint32_t now) {
    float measured[2]={vl,vr},u[2]={0,0};
    for(unsigned j=0;j<2;++j) {
        int sign=requested[j]>0?1:requested[j]<0?-1:0;
        if(!sign) {
            if(!zero_valid[j]) { zero_since[j]=now; zero_valid[j]=true; }
            W4Pid_Reset(&pid[j]); continue;
        }
        if(sign!=direction[j]) {
            if(!zero_valid[j]) { zero_valid[j]=true; zero_since[j]=now; }
            W4Pid_Reset(&pid[j]);
            if(now-zero_since[j]<50) continue; /* nonblocking reversal */
            direction[j]=sign;
        }
        zero_valid[j]=false;
        /* PID output limit includes feedforward/deadzone budget. */
        float ff=clamp(w4_config.wheel_ff*requested[j]+sign*w4_config.pwm_deadzone,w4_config.pwm_limit);
        float margin=fmaxf(0.001f,w4_config.pwm_limit-fabsf(ff));
        float correction=W4Pid_Update(&pid[j],requested[j]-measured[j],dt,
            w4_config.wheel_kp,w4_config.wheel_ki,w4_config.wheel_kd,margin);
        u[j]=clamp(ff+correction,w4_config.pwm_limit);
        /* Never electrically reverse due to an overspeed PID correction. */
        if(u[j]*sign<0) u[j]=0;
    }
    /* PID arithmetic runs with IRQs enabled. Only the final STOP check and
       hardware commit need the mask, preventing an old calculation after STOP. */
    uint32_t key=W4Port_Lock();
    if(w4_stop_pending) W4Port_WritePwm(0,0);
    else W4Port_WritePwm(u[0],u[1]);
    W4Port_Unlock(key);
}
