#include "w4_nav.h"
#include "w4_rtos.h"
#include "w4_runtime.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#define RX_SIZE 512U
#define RX_EVENT 1U
static uint8_t ring[RX_SIZE];
static volatile uint32_t head,tail,loss_epoch;
static bool isr_drop;
static osThreadId_t rx_thread;
static char line[64];
static unsigned used;
static bool dropping;
static uint32_t seen_epoch,next_id;
/* Decimal-only grammar: rejects whitespace, hex floats, NaN, Inf, junk. */
static bool number(const char **p,float *out) {
    const char *s=*p,*q=s; unsigned digits=0;
    if(*q=='+'||*q=='-') ++q;
    while(*q>='0'&&*q<='9') { ++q; ++digits; }
    if(*q=='.') { ++q; while(*q>='0'&&*q<='9') { ++q; ++digits; } }
    if(!digits) return false;
    if(*q=='e'||*q=='E') {
        ++q; if(*q=='+'||*q=='-') ++q; digits=0;
        while(*q>='0'&&*q<='9') { ++q; ++digits; } if(!digits) return false;
    }
    char *end; float v=strtof(s,&end);
    if(end!=q||!isfinite(v)||fabsf(v)>5) return false;
    *out=v; *p=q; return true;
}
bool W4ParseCommand(const char *s,W4Command *c) {
    memset(c,0,sizeof *c);
    if(!strcmp(s,"STOP")) { c->type=W4_CMD_STOP; return true; }
    bool maze=false;
#ifdef W4_MAZE
    if(!strcmp(s,"RETURN")) { c->type=W4_CMD_RETURN; c->x_m=0.0f; c->y_m=0.0f; return true; }
    maze=!strncmp(s,"MAZE,",5);
#endif
    if(!maze && strncmp(s,"GOTO,",5)) return false;
    s+=5;
    if(!number(&s,&c->x_m)||*s++!=','||!number(&s,&c->y_m)||*s) return false;
    c->type=maze?W4_CMD_MAZE:W4_CMD_GOTO; return true;
}
void W4Bt_OnByteISR(uint8_t b) {
    if(isr_drop) { if(b=='\n') isr_drop=false; }
    else if((uint32_t)(head-tail)>=RX_SIZE) {
        ++loss_epoch; ++w4_stats.rx_overflow; isr_drop=(b!='\n');
    } else { ring[head%RX_SIZE]=b; ++head; }
    if(rx_thread) (void)osThreadFlagsSet(rx_thread,RX_EVENT);
}
void W4Bt_OnErrorISR(void) {
    ++w4_stats.rx_errors; ++loss_epoch; isr_drop=true;
    if(rx_thread) (void)osThreadFlagsSet(rx_thread,RX_EVENT);
}
static void dispatch(void) {
    W4Command c;
#ifdef W4_REAR_FORWARD
    if(!strncmp(line,"SPEED,",6)) {
        const char *p=line+6;
        float value;
        if(++next_id==0) ++next_id;
        if(!number(&p,&value)||*p||value<0.08f||value>w4_config.max_v) {
            W4Ack(next_id,"REJECTED","SPEED_RANGE"); return;
        }
        uint32_t key=W4Port_Lock();
        w4_cruise_request=value;
        W4Port_Unlock(key);
        char reason[32];
        snprintf(reason,sizeof reason,"SPEED_MMPS_%ld",lroundf(1000*value));
        W4Ack(next_id,"OK",reason); return;
    }
    if(!strcmp(line,"CAL?") || !strncmp(line,"CAL,",4)) {
        if(++next_id==0) ++next_id;
        bool query=!strcmp(line,"CAL?");
        bool off=!strcmp(line,"CAL,OFF");
        float left=0,right=0;
        if(!query && !off) {
            const char *p=line+4;
            if(!number(&p,&left)||*p++!=','||!number(&p,&right)||*p||
               left<=0||right<=0||left>w4_config.pwm_limit||right>w4_config.pwm_limit) {
                W4Ack(next_id,"REJECTED","CAL_RANGE"); return;
            }
        }
        uint32_t key=W4Port_Lock();
        bool ok=query || W4Control_CalibrationReady(W4Port_Millis());
        if(ok && !query) {
            if(!off) { w4_straight_cal.base[0]=left; w4_straight_cal.base[1]=right; }
            w4_straight_cal.enabled=!off;
        }
        W4StraightCalibration cal=w4_straight_cal;
        W4Port_Unlock(key);
        if(!ok) { W4Ack(next_id,"REJECTED","CAL_REQUIRES_STILL"); return; }
        char reason[48];
        snprintf(reason,sizeof reason,"CAL_%s_L%ld_R%ld",cal.enabled?"ON":"OFF",
                 lroundf(10000*cal.base[0]),lroundf(10000*cal.base[1]));
        W4Ack(next_id,"OK",reason); return;
    }
#endif
    /* RESET is a local stopped-pose operation; keep the shared command enum frozen. */
    if(!strcmp(line,"RESET")) {
        if(++next_id==0) ++next_id;
        if(W4Nav_ResetPose(0,0,W4_PI/2)) W4Ack(next_id,"OK","RESET_PENDING");
        else W4Ack(next_id,"REJECTED","RESET_REQUIRES_STOP_AND_IMU");
        return;
    }
    if(!W4ParseCommand(line,&c)) { ++w4_stats.rx_invalid; W4Ack(0,"REJECTED","BAD_COMMAND"); return; }
    if(++next_id==0) ++next_id;
    c.cmd_id=next_id;
    if(c.type==W4_CMD_STOP) {
        W4Nav_RequestStop(); /* independent of queue availability */
        (void)osMessageQueuePut(Q_Cmd,&c,0,0);
        W4Ack(c.cmd_id,"STOPPED","STOP_LATCHED"); return;
    }
    uint32_t k=W4Port_Lock();
    bool busy=w4_stop_pending||w4_reset_pending||w4_nav.state==W4_PLANNING||w4_nav.state==W4_TRACKING;
#ifdef W4_MAZE
    /* RETURN can preempt an active MAZE mission cleanly to head back to (0,0). */
    if(c.type==W4_CMD_RETURN) busy=w4_stop_pending||w4_reset_pending;
#endif
    if(!busy) {
        w4_nav.state=W4_PLANNING; w4_nav.cmd_id=c.cmd_id;
        w4_nav.target_x=c.x_m; w4_nav.target_y=c.y_m; w4_nav.fault=0;
    }
    W4Port_Unlock(k);
    if(busy) { W4Ack(c.cmd_id,"BUSY","NAV_ACTIVE"); return; }
    if(osMessageQueuePut(Q_Cmd,&c,0,0)!=osOK) {
        ++w4_stats.cmd_full;
        k=W4Port_Lock(); w4_nav.state=W4_IDLE; W4Port_Unlock(k);
        W4Ack(c.cmd_id,"REJECTED","CMD_QUEUE_FULL");
    }
}
void W4Bt_Process(void) {
    for(;;) {
        uint32_t k=W4Port_Lock();
        if(seen_epoch!=loss_epoch) {
            seen_epoch=loss_epoch; tail=head; used=0;
            /* ISR already discards until LF while isr_drop is true. If it
               has resumed, the flushed queue may contain a new line prefix. */
            dropping=!isr_drop;
        }
        bool empty=(tail==head); uint8_t b=0;
        if(!empty) b=ring[(tail++)%RX_SIZE];
        W4Port_Unlock(k);
        if(empty) break;
        if(dropping) { if(b=='\n') { dropping=false; used=0; } continue; }
        if(b=='\n') {
            if(used && line[used-1]=='\r') --used;
            line[used]=0; dispatch(); used=0;
        } else if(b==0 || used>=sizeof(line)-1) {
            /* at most 64 bytes including LF; reserve a NUL for parsing */
            ++w4_stats.rx_long; dropping=true; used=0; W4Ack(0,"REJECTED","LINE_TOO_LONG");
        } else line[used++]=(char)b;
    }
}
void W4BluetoothRxTask(void *argument) {
    (void)argument; rx_thread=osThreadGetId();
    bool started=W4Port_RxStart(); uint32_t errors=w4_stats.rx_errors;
    for(;;) {
        (void)osThreadFlagsWait(RX_EVENT,osFlagsWaitAny,started?osWaitForever:10);
        W4Runtime_Begin(W4_TASK_RX,osKernelGetTickCount());
        W4Bt_Process();
        if(!started||errors!=w4_stats.rx_errors) {
            errors=w4_stats.rx_errors; started=W4Port_RxRecover();
        }
        W4Runtime_End(W4_TASK_RX,0);
    }
}
