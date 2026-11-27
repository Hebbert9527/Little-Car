#include "w4_period.h"
#include <stddef.h>
bool W4Tick_After(uint32_t a,uint32_t b) { return (int32_t)(a-b)>0; }
uint32_t W4Period_Next(uint32_t previous,uint32_t now,uint32_t period,uint32_t *skipped)
{
    if (!period || period>=0x80000000U) { if (skipped) *skipped=0; return previous; }
    uint32_t next=previous+period;
    uint32_t n=0;
    if (W4Tick_After(now,next)) {
        n=(uint32_t)(now-next)/period+1;
        next+=n*period;
    }
    if (skipped) *skipped=n;
    return next;
}
bool W4Period_Init(W4Period *p,uint32_t period)
{
    if (!p || !period || period>=0x80000000U) return false;
    *p=(W4Period){osKernelGetTickCount(),period,0};
    return true;
}
osStatus_t W4Period_Wait(W4Period *p)
{
    if (!p || !p->period || p->period>=0x80000000U) return osErrorParameter;
    uint32_t skipped;
    uint32_t next=W4Period_Next(p->release,osKernelGetTickCount(),p->period,&skipped);
    p->skipped+=skipped;
    for (;;) {
        uint32_t now=osKernelGetTickCount();
        if (now==next) { p->release=next; return osOK; }
        if (W4Tick_After(now,next)) {
            uint32_t n=(uint32_t)(now-next)/p->period+1;
            p->skipped+=n; next+=n*p->period;
        }
        osStatus_t s=osDelayUntil(next);
        if (s==osOK) { p->release=next; return s; }
        if (s!=osErrorParameter) return s;
        /* The caller may have been preempted between checking and delaying. */
    }
}
uint32_t W4Period_Remaining(uint32_t deadline,uint32_t now)
{
    return W4Tick_After(deadline,now) ? deadline-now : 0;
}
