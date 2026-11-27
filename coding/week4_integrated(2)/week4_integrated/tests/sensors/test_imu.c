#include "w4_imu_rtos.h"
#include "w4_runtime.h"
#include "stm32f4xx_hal.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

osMutexId_t Mtx_I2C1=(void *)1;
static uint32_t ticks, reads, writes, acquires, legacy_reads;
static unsigned regs;
static jmp_buf done;
uint32_t __get_PRIMASK(void){return 0;}
void __disable_irq(void){}
void __set_PRIMASK(uint32_t n){(void)n;}
uint32_t osKernelGetTickCount(void){return ticks;}
uint32_t HAL_GetTick(void){return ticks+400;}
uint32_t osKernelGetTickFreq(void){return 1000;}
osThreadId_t osThreadGetId(void){return (void *)2;}
osSemaphoreId_t osSemaphoreNew(uint32_t max,uint32_t initial,const osSemaphoreAttr_t *a){(void)a;assert(max==1&&initial==0);return (void *)3;}
osStatus_t osSemaphoreRelease(osSemaphoreId_t s){assert(s==Sem_ImuTick);return osOK;}
osStatus_t osSemaphoreAcquire(osSemaphoreId_t s,uint32_t timeout){
 (void)timeout;assert(s==Sem_ImuTick);ticks+=10;
 if(++acquires>
#ifdef TEST_RECOVERY
 430
#else
 215
#endif
 )longjmp(done,1);
 return osOK;
}
osStatus_t osMutexAcquire(osMutexId_t m,uint32_t t){(void)m;(void)t;return osOK;}
osStatus_t osMutexRelease(osMutexId_t m){(void)m;return osOK;}
uint32_t osThreadFlagsSet(osThreadId_t t,uint32_t f){(void)t;return f;}
osStatus_t osDelay(uint32_t n){ticks+=n;return osOK;}
void W4Runtime_Begin(W4TaskId id,uint32_t at){assert(id==W4_TASK_IMU);(void)at;}
void W4Runtime_End(W4TaskId id,uint32_t deadline){assert(id==W4_TASK_IMU);(void)deadline;}
static void stationary(uint8_t *b){memset(b,0,14);b[4]=0x40;}
bool W4I2c_MemRead(uint8_t addr,uint8_t reg,uint8_t *b,uint16_t len,uint32_t timeout){
 assert((addr==0x68||addr==0x69)&&timeout>0);
 if(reg==0x75){assert(len==1);*b=0x70;return true;}
 assert(reg==0x3b&&len==14);assert(regs==31);++reads;stationary(b);
#ifdef TEST_RECOVERY
 if(reads>=211&&reads<=213)return false;
 if(reads>=201&&reads<=210)b[13]=131; /* 1 degree/s before the read failures. */
#endif
 return true;
}
bool W4I2c_MemWrite(uint8_t addr,uint8_t reg,const uint8_t *b,uint16_t len,uint32_t timeout){
 assert(addr==0x68&&len==1&&timeout>0);++writes;
 switch(reg){case 0x6b:assert(*b==0);regs|=1;break;case 0x1b:assert(*b==0);regs|=2;break;
 case 0x1c:assert(*b==0);regs|=4;break;case 0x1a:assert(*b==3);regs|=8;break;
 case 0x19:assert(*b==9);regs|=16;break;default:assert(0);}
 return true;
}
bool W4I2c_Recover(uint32_t timeout){(void)timeout;return true;}
osStatus_t W4I2C_ReadMpu6500(uint8_t a,uint8_t r,uint8_t *b,uint16_t n,uint32_t t){
 (void)a;(void)r;(void)n;(void)t;++legacy_reads;stationary(b);return osOK;
}
int main(void){
 assert(W4ImuSync_Init()==osOK);
 if(setjmp(done)==0)W4ImuTask(NULL);
 W4ImuSnapshot s;assert(W4Imu_GetSnapshotRaw(&s));
 assert(reads>=200&&legacy_reads==0);
#ifdef TEST_RECOVERY
 assert(writes==10&&s.calibrated&&s.valid&&s.heading_rad>0.001f&&s.gz_rad_s==0);
#else
 assert(writes==5&&s.calibrated&&s.valid&&s.heading_rad==0.0f&&s.gz_rad_s==0.0f);
#endif
 assert(s.sample_ms>=400&&s.sample_ms<=HAL_GetTick());
 puts("PASS IMU shared-bus initialization, calibration and relative heading");return 0;
}
