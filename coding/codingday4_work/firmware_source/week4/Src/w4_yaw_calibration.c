#include "w4_yaw_calibration.h"
#include <math.h>
#include <string.h>

void W4YawCal_Reset(W4YawCalibration *c){memset(c,0,sizeof *c);}
static W4YawCalResult reject(W4YawCalibration *c,W4YawCalReason reason,uint32_t now)
{
    W4YawCal_Reset(c);c->last_ms=now;c->have_time=1;c->reason=(uint8_t)reason;
    return W4_YAW_REJECTED;
}
W4YawCalResult W4YawCal_Push(W4YawCalibration *c,const Mpu6500Sample *s,uint32_t now)
{
    if(c->samples>=W4_YAW_CAL_SAMPLES)return W4_YAW_READY;
    float v[6]={s->ax_g,s->ay_g,s->az_g,s->gx_dps,s->gy_dps,s->gz_dps};
    for(unsigned j=0;j<6;++j)if(!isfinite(v[j]))return reject(c,W4_YAW_INVALID,now);
    /* This chassis measured -6.92 dps while confirmed stationary.
       Require a full quiet window and keep range, variance and gap checks. */
    float norm2=v[0]*v[0]+v[1]*v[1]+v[2]*v[2];
    if(fabsf(v[0])>0.25f||fabsf(v[1])>0.25f||fabsf(v[2])<0.70f||fabsf(v[2])>1.30f||
       norm2<0.85f*0.85f||norm2>1.15f*1.15f||fabsf(v[5])>10.0f||
       fabsf(v[3])>=245.0f||fabsf(v[4])>=245.0f)
        return reject(c,W4_YAW_MOVING,now);
    if(c->have_time){
        uint32_t dt=now-c->last_ms;
        if(!dt||dt>30U)return reject(c,W4_YAW_GAP,now);
    }
    c->have_time=1;c->last_ms=now;
    uint32_t n=c->samples+1;
    for(unsigned j=0;j<6;++j){
        if(n==1)c->minimum[j]=c->maximum[j]=v[j];
        if(v[j]<c->minimum[j])c->minimum[j]=v[j];
        if(v[j]>c->maximum[j])c->maximum[j]=v[j];
        float span_limit=j<3?0.12f:(j==5?1.0f:2.0f);
        if(c->maximum[j]-c->minimum[j]>span_limit)return reject(c,W4_YAW_MOVING,now);
        float delta=v[j]-c->mean[j];c->mean[j]+=delta/(float)n;
        c->m2[j]+=delta*(v[j]-c->mean[j]);
    }
    c->samples=n;
    if(n<W4_YAW_CAL_SAMPLES)return W4_YAW_PENDING;
    /* Limits apply to fluctuations around each axis mean, so stable unused
       X/Y offsets do not suppress an otherwise measurable planar Z heading. */
    for(unsigned j=0;j<6;++j){
        float sd_limit=j<3?0.02f:0.25f;
        if(c->m2[j]/(float)(n-1)>sd_limit*sd_limit)return reject(c,W4_YAW_NOISY,now);
    }
    c->bias_dps=c->mean[5];
    c->xy_bias_warning=fabsf(c->mean[3])>5.0f||fabsf(c->mean[4])>5.0f;
    c->reason=W4_YAW_NONE;
    return W4_YAW_READY;
}
