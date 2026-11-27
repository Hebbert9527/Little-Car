/* TEST ONLY. Never add this file to CubeIDE firmware. */
#include "w4_nav.h"
#include "w4_rtos.h"
#include "w4_runtime.h"
#include <string.h>
typedef struct { unsigned count,read,write,cap,size; uint8_t data[64][32]; } Queue;
static Queue commands={.cap=8,.size=sizeof(W4Command)},path={.cap=64,.size=sizeof(W4Waypoint)};
osMessageQueueId_t Q_Cmd=&commands,Q_Path=&path;
uint32_t test_ms;
uint32_t test_advance_ms;
uint32_t test_imu_advance_ms;
float test_pwm_l,test_pwm_r;
bool test_ready=true,test_imu_valid=true;
float test_heading,test_gyro;
int w4_single_cmd_dir;
void W4Runtime_Begin(W4TaskId id,uint32_t release) { (void)id; (void)release; }
void W4Runtime_End(W4TaskId id,uint32_t deadline) { (void)id; (void)deadline; }
char test_ack[96];
osStatus_t osMessageQueuePut(osMessageQueueId_t q,const void *p,uint8_t pri,uint32_t wait) {
    (void)pri;(void)wait; Queue *x=q;
    if(x->count==x->cap) return osErrorResource;
    memcpy(x->data[x->write++%x->cap],p,x->size); ++x->count; return osOK;
}
osStatus_t osMessageQueueGet(osMessageQueueId_t q,void *p,uint8_t *pri,uint32_t wait) {
    (void)pri;(void)wait; Queue *x=q;
    if(!x->count) return osErrorResource;
    memcpy(p,x->data[x->read++%x->cap],x->size); --x->count; return osOK;
}
osStatus_t osMessageQueueReset(osMessageQueueId_t q) { Queue *x=q; x->count=x->read=x->write=0; return osOK; }
uint32_t osMessageQueueGetCount(osMessageQueueId_t q) { return ((Queue*)q)->count; }
osThreadId_t osThreadGetId(void) { return (void*)1; }
uint32_t osThreadFlagsSet(osThreadId_t id,uint32_t flags) { (void)id; return flags; }
uint32_t osThreadFlagsWait(uint32_t f,uint32_t o,uint32_t t) { (void)o;(void)t; return f; }
uint32_t osKernelGetTickFreq(void) { return 1000; }
uint32_t osKernelGetTickCount(void) { return test_ms; }
osStatus_t osDelay(uint32_t n) { test_ms+=n; return osOK; }
osStatus_t osDelayUntil(uint32_t n) { test_ms=n; return osOK; }
uint32_t W4Port_Lock(void) { return 0; }
void W4Port_Unlock(uint32_t k) { (void)k; }
uint32_t W4Port_Millis(void) { test_ms+=test_advance_ms; test_advance_ms=0; return test_ms; }
uint32_t W4Port_Micros(void) { return test_ms*1000; }
bool W4Port_EncodersReady(void) { return test_ready; }
void W4Port_WritePwm(float l,float r) { test_pwm_l=l; test_pwm_r=r; w4_single_cmd_dir=r>0?1:r<0?-1:0; }
bool W4Port_RxStart(void) { return true; }
bool W4Port_RxRecover(void) { return true; }
bool W4Imu_GetSnapshot(W4ImuSample *p) {
    test_ms+=test_imu_advance_ms; test_imu_advance_ms=0;
    *p=(W4ImuSample){test_ms,test_heading,test_gyro,test_imu_valid,1}; return test_imu_valid;
}
bool W4Telemetry_SubmitLine(const char *p,uint16_t n) {
    if(n>=sizeof test_ack) return false;
    memcpy(test_ack,p,n); test_ack[n]=0; return true;
}
