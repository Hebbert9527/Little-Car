#include "w4_imu_rtos.h"
#include "w4_runtime.h"
#include "stm32f4xx_hal.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#ifdef W4_IMU_DIAGNOSTICS
#include "w4_imu_diag.h"
#endif

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
#elif defined(TEST_POST_CAL_ROTATION)
 301
#elif defined(W4_IMU_DIAGNOSTICS)
 515
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
void W4Runtime_End(W4TaskId id,uint32_t deadline){
 assert(id==W4_TASK_IMU);(void)deadline;
#ifdef TEST_RECOVERY
 if(reads==201||reads==211){
  W4ImuSnapshot sample;
  assert(W4Imu_GetSnapshotRaw(&sample)&&sample.valid&&sample.calibrated);
  assert(HAL_GetTick()-sample.sample_ms==10);
 }else if(reads==202||reads==214){
  W4ImuSnapshot sample;
  bool valid=W4Imu_GetSnapshotRaw(&sample)!=0;
  assert(valid==(reads==202));
  assert(sample.sample_ms==HAL_GetTick());
 }else if(reads==212||reads==213){
  W4ImuSnapshot sample;
  assert(!W4Imu_GetSnapshotRaw(&sample)&&!sample.valid);
 }
#endif
}
static void stationary(uint8_t *b){
 memset(b,0,14);b[4]=0x40;
#ifdef TEST_XY_BIAS
 b[8]=2;b[9]=222; /* 734/131 = 5.60 degrees/s */
 b[10]=7;b[11]=101; /* 1893/131 = 14.45 degrees/s */
#endif
}
bool W4I2c_MemRead(uint8_t addr,uint8_t reg,uint8_t *b,uint16_t len,uint32_t timeout){
 assert((addr==0x68||addr==0x69)&&timeout>0);
 if(reg==0x75){assert(len==1);*b=0x70;return true;}
#ifdef W4_IMU_DIAGNOSTICS
 if(reg==0x13){assert(len==10);memset(b,0,len);if(regs==31){b[6]=9;b[7]=3;}return true;}
 if(reg==0x6b){assert(len==2);memset(b,0,len);return true;}
#endif
 assert(reg==0x3b&&len==14);assert(regs==31);++reads;stationary(b);
#ifdef TEST_POST_CAL_ROTATION
 {int16_t z=reads<=200?131:11921;b[12]=(uint16_t)z>>8;b[13]=(uint8_t)z;}
#endif
#ifdef TEST_RECOVERY
 if(reads==201||(reads>=211&&reads<=213))return false;
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
 W4ImuSnapshot s;
#ifdef TEST_XY_BIAS
 assert(W4Imu_GetSnapshotRaw(&s)&&s.valid&&s.calibrated&&s.xy_bias_warning);
 assert(W4Imu_Counters.calibration_motion==0&&W4Imu_Counters.calibration_samples==200);
#else
 assert(W4Imu_GetSnapshotRaw(&s));
#endif
 assert(reads>=200&&legacy_reads==0);
#ifdef TEST_RECOVERY
 assert(writes==10&&s.calibrated&&s.valid&&s.heading_rad>0.001f&&s.gz_rad_s==0);
 assert(W4Imu_Counters.failed==4);
#elif defined(TEST_POST_CAL_ROTATION)
 assert(writes==5&&s.calibrated&&s.valid&&fabsf(s.heading_rad-1.5707963268f)<0.001f);
#elif !defined(TEST_XY_BIAS)
 assert(writes==5&&s.calibrated&&s.valid&&s.heading_rad==0.0f&&s.gz_rad_s==0.0f);
#endif
#ifdef W4_IMU_DIAGNOSTICS
 assert(W4ImuDiag.ready==2&&W4ImuDiag.samples==500&&W4ImuDiag.configurations==1);
 assert(W4ImuDiag.magic==0x494d5531U&&W4ImuDiag.read_mask==63);
 assert(W4ImuDiag.after[6]==9&&W4ImuDiag.after[7]==3&&W4ImuDiag.after[8]==0&&W4ImuDiag.after[12]==0x70);
 assert(fabsf(W4ImuDiag.mean[2]-1)<1e-6f&&W4ImuDiag.m2[2]==0);
#ifdef TEST_XY_BIAS
 assert(W4ImuDiag.mean[3]>5&&W4ImuDiag.mean[4]>14&&W4ImuDiag.mean[5]==0);
#endif
#endif
 assert(s.sample_ms>=400&&s.sample_ms<=HAL_GetTick());
 puts("PASS IMU shared-bus initialization, planar yaw calibration and relative heading");return 0;
}
