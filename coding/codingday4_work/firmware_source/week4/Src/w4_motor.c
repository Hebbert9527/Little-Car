#include "w4_nav.h"
#ifdef W4_MAZE
#include "w4_maze.h"
#endif
#include <math.h>
#ifdef W4_DUAL_ENCODER
#include "w4_wheel_speed.h"
#ifdef W4_WHEEL_DIAGNOSTICS
#include <stdio.h>
#endif
#endif

static float requested[2];
static int direction[2];
static uint32_t zero_since[2];
static bool zero_valid[2];
#ifdef W4_DUAL_ENCODER
static float filtered_speed[2], startup_seconds[2], quiet_seconds[2], wheel_integral[2], travel_error_m;
static bool wheel_started[2];
#ifndef W4_DISABLE_STRAIGHT_CORRECTION
static W4Pid wheel_difference_pid;
static float differential_duty;
#endif
/* Mean duty during the matched 90 RPM bench runs, by wheel. */
static const float duty_at_90_rpm[2] = {0.272f,0.320f};
#ifdef W4_REAR_FORWARD
static float left_pivot_assist_duty;
static const float left_pivot_assist_limit=0.60f;
/* Ground run 26 gives the loaded duty-to-speed estimate. On the 0.18 m/s
   run 8 the left wheel was still 4.39 mm/s faster. At that operating point,
   remove 0.8 percentage points of left feedforward and keep the right value. */
static const float ground_duty_per_mps[2] = {
    0.3620f/0.20497f - 0.008f/0.18f,
    0.3893f/0.19687f,
};
#endif
#ifdef W4_WHEEL_DIAGNOSTICS
typedef struct {
    uint32_t ms, cmd_id;
    float request[2], target[2], measured[2], duty[2];
    bool valid;
} W4WheelDiagnostics;

static W4WheelDiagnostics diagnostics;

uint16_t W4Motor_FormatDiagnostics(char *out, uint16_t capacity)
{
    uint32_t key = W4Port_Lock();
    W4WheelDiagnostics sample = diagnostics;
    W4Port_Unlock(key);
    if (!sample.valid) return 0;

    /* Logical left/right, signed mm/s and signed duty in thousandths. */
    int length = snprintf(out, capacity, "D,%lu,WHEEL,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
        (unsigned long)sample.ms, (unsigned long)sample.cmd_id,
        lroundf(1000.0f*sample.request[0]), lroundf(1000.0f*sample.request[1]),
        lroundf(1000.0f*sample.target[0]), lroundf(1000.0f*sample.target[1]),
        lroundf(1000.0f*sample.measured[0]), lroundf(1000.0f*sample.measured[1]),
        lroundf(1000.0f*sample.duty[0]), lroundf(1000.0f*sample.duty[1]));
    return length > 0 && length < capacity ? (uint16_t)length : 0;
}
#endif
#else
static W4Pid pid[2];
#endif

static float clamp(float x,float a) { return fmaxf(-a,fminf(a,x)); }
static void reset_wheel(unsigned wheel) {
#ifdef W4_DUAL_ENCODER
    filtered_speed[wheel]=startup_seconds[wheel]=quiet_seconds[wheel]=wheel_integral[wheel]=0;
    wheel_started[wheel]=false;
#else
    W4Pid_Reset(&pid[wheel]);
#endif
}
void W4Motor_SetWheelSpeeds(float l,float r) {
    if(!isfinite(l)||!isfinite(r)) { requested[0]=requested[1]=0; return; }
    requested[0]=clamp(l,w4_config.max_wheel); requested[1]=clamp(r,w4_config.max_wheel);
}
void W4Motor_Stop(void) {
    uint32_t now=W4Port_Millis();
    requested[0]=requested[1]=0;
#ifdef W4_DUAL_ENCODER
    travel_error_m=0;
#ifdef W4_REAR_FORWARD
    left_pivot_assist_duty=0.0f;
#endif
#ifndef W4_DISABLE_STRAIGHT_CORRECTION
    W4Pid_Reset(&wheel_difference_pid);
    differential_duty=0.0f;
#endif
#endif
    for(unsigned j=0;j<2;++j) {
        reset_wheel(j);
        if(!zero_valid[j]) { zero_since[j]=now; zero_valid[j]=true; }
    }
#if defined(W4_DUAL_ENCODER) && defined(W4_WHEEL_DIAGNOSTICS)
    uint32_t key=W4Port_Lock();
    diagnostics.valid=false;
    W4Port_Unlock(key);
#endif
    W4Port_WritePwm(0,0);
}
void W4Motor_Update(float vl,float vr,float dt,uint32_t now,bool fixed_distance) {
    float measured[2]={vl,vr},u[2]={0,0};
#ifdef W4_DUAL_ENCODER
    float target[2]={requested[0],requested[1]};
#ifdef W4_REAR_FORWARD
    const bool left_pivot=requested[0]<-0.03f&&requested[1]>0.03f;
    if(!left_pivot) left_pivot_assist_duty=0.0f;
#endif
    if(!fixed_distance && requested[0]*requested[1]>0.0f && dt>0.0f && dt<=0.03f) {
        /* Track the requested wheel differential to retain deliberate steering. */
        travel_error_m=clamp(travel_error_m+
            ((vl-vr)-(requested[0]-requested[1]))*dt,0.07f);
        float limit=0.5f*fminf(fabsf(requested[0]),fabsf(requested[1]));
        float trim=clamp(0.75f*travel_error_m,limit);
        target[0]=clamp(requested[0]-trim,w4_config.max_wheel);
        target[1]=clamp(requested[1]+trim,w4_config.max_wheel);
    } else {
        travel_error_m=0;
    }
#else
    (void)fixed_distance;
#endif
    for(unsigned j=0;j<2;++j) {
        int sign=requested[j]>0?1:requested[j]<0?-1:0;
        if(!sign) {
            if(!zero_valid[j]) { zero_since[j]=now; zero_valid[j]=true; }
            reset_wheel(j);
            continue;
        }
        if(sign!=direction[j]) {
            if(!zero_valid[j]) { zero_valid[j]=true; zero_since[j]=now; }
            reset_wheel(j);
            if(now-zero_since[j]<50) continue;
#ifdef W4_DUAL_ENCODER
            if(fabsf(measured[j])>0.02f) continue;
#endif
            direction[j]=sign;
        }
        zero_valid[j]=false;
#ifdef W4_DUAL_ENCODER
        filtered_speed[j]+=dt/(0.10f+dt)*(measured[j]-filtered_speed[j]);
        wheel_started[j]=wheel_started[j]||sign*filtered_speed[j]>=0.018f;
        if(wheel_started[j]&&fabsf(requested[j])>=0.03f&&fabsf(measured[j])<0.003f) {
            quiet_seconds[j]+=dt;
            if(quiet_seconds[j]>=0.15f&&sign*filtered_speed[j]<0.018f) reset_wheel(j);
        } else {
            quiet_seconds[j]=0.0f;
        }
        if(!wheel_started[j]) startup_seconds[j]+=dt;
        float diameter=j==0?w4_config.diameter_l_m:w4_config.diameter_r_m;
        float rpm_per_mps=60.0f/(W4_PI*diameter);
        float target_rpm=fabsf(target[j])*rpm_per_mps;
#ifdef W4_REAR_FORWARD
        unsigned physical_motor=1u-j;
#else
        unsigned physical_motor=j;
#endif
        float feedforward=0.10f+(duty_at_90_rpm[physical_motor]-0.10f)*target_rpm/90.0f;
#ifdef W4_REAR_FORWARD
        /* The ground estimate was measured near 0.18 m/s. Let PI supply
           the remaining duty at higher targets. */
        if(fixed_distance) feedforward=ground_duty_per_mps[j]*fminf(fabsf(target[j]),0.18f);
        if(fixed_distance && (w4_straight_cal.enabled
#ifdef W4_MAZE
            || W4Maze_Active(w4_nav.cmd_id)
#endif
            ))
            feedforward=w4_straight_cal.base[j]*fminf(fabsf(target[j])/0.18f,1.0f);
        /* Extra loaded feedforward only above the original alignment speed.
           It fades during the fine turn; the existing duty limits still apply. */
        if(requested[0]*requested[1]<0.0f)
            feedforward+=fminf(0.10f,2.0f*fmaxf(0.0f,fabsf(target[j])-0.088f));
#endif
        float duty=W4WheelSpeed_CommandDuty(target_rpm,sign*filtered_speed[j]*rpm_per_mps,
            dt,feedforward,&wheel_integral[j]);
        u[j]=sign*fminf(duty,w4_config.pwm_limit);
        if(!wheel_started[j]&&fabsf(requested[j])>=0.03f&&startup_seconds[j]<1.0f) {
            if(physical_motor==1u) u[j]=sign*w4_config.pwm_limit;
            else if(startup_seconds[j]>=0.15f) u[j]=sign*0.60f;
        }
#ifdef W4_REAR_FORWARD
        if(left_pivot&&j==0) {
            float target_speed=fabsf(target[j]);
            float measured_speed=sign*filtered_speed[j];
            /* Under load the original right motor lost speed when the assist
               ended at the first encoder feedback. Hold it until tracking. */
            if(left_pivot_assist_duty==0.0f&&startup_seconds[j]>=0.15f)
                left_pivot_assist_duty=left_pivot_assist_limit;
            if(left_pivot_assist_duty>0.0f) {
                if(measured_speed>=target_speed)
                    left_pivot_assist_duty=fmaxf(0.0f,left_pivot_assist_duty-0.65f*dt);
                else if(measured_speed<0.8f*target_speed)
                    left_pivot_assist_duty=fminf(left_pivot_assist_limit,left_pivot_assist_duty+1.0f*dt);
                u[j]=sign*fmaxf(fabsf(u[j]),left_pivot_assist_duty);
            }
        }
#endif
#else
        /* PID output limit includes feedforward/deadzone budget. */
        float ff=clamp(w4_config.wheel_ff*requested[j]+sign*w4_config.pwm_deadzone,w4_config.pwm_limit);
        float margin=fmaxf(0.001f,w4_config.pwm_limit-fabsf(ff));
        float correction=W4Pid_Update(&pid[j],requested[j]-measured[j],dt,
            w4_config.wheel_kp,w4_config.wheel_ki,w4_config.wheel_kd,margin);
        u[j]=clamp(ff+correction,w4_config.pwm_limit);
        if(u[j]*sign<0) u[j]=0;
#endif
    }
#if defined(W4_DUAL_ENCODER) && !defined(W4_DISABLE_STRAIGHT_CORRECTION)
    if(requested[0]>0.0f&&requested[1]>0.0f) {
        float wanted=0.0f;
        float limit=w4_config.pwm_limit;
        bool saturated=fmaxf(u[0],u[1])>=limit-0.01f;
        bool moving=filtered_speed[0]>=0.06f&&filtered_speed[1]>=0.06f;
        if(saturated&&moving) {
            /* When one wheel cannot get more power, slow the other wheel
               just enough to track the requested speed difference. */
            float error=(requested[0]-requested[1])-(filtered_speed[0]-filtered_speed[1]);
            wanted=W4Pid_Update(&wheel_difference_pid,error,dt,0.9f,1.15f,0.0f,0.07f);
        } else {
            W4Pid_Reset(&wheel_difference_pid);
        }
        differential_duty+=clamp(wanted-differential_duty,0.12f*dt);
        if(moving) {
            u[0]=fmaxf(0.0f,u[0]-fmaxf(0.0f,-differential_duty));
            u[1]=fmaxf(0.0f,u[1]-fmaxf(0.0f,differential_duty));
        }
    } else {
        W4Pid_Reset(&wheel_difference_pid);
        differential_duty=0.0f;
    }
#endif
    /* Check STOP and commit both PWM values in one critical section. */
    uint32_t key=W4Port_Lock();
    if(w4_stop_pending) W4Port_WritePwm(0,0);
#ifdef W4_REAR_FORWARD
    else W4Port_WritePwm(-u[1],-u[0]);
#else
    else W4Port_WritePwm(u[0],u[1]);
#endif
#if defined(W4_DUAL_ENCODER) && defined(W4_WHEEL_DIAGNOSTICS)
    diagnostics = (W4WheelDiagnostics){
        .ms=now, .cmd_id=w4_nav.cmd_id,
        .request={requested[0], requested[1]}, .target={target[0], target[1]},
        .measured={filtered_speed[0], filtered_speed[1]}, .duty={u[0], u[1]},
        .valid=!w4_stop_pending && w4_nav.state==W4_TRACKING
    };
#endif
    W4Port_Unlock(key);
}
