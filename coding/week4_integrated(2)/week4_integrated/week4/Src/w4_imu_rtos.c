#include "w4_imu_rtos.h"
#include "w4_i2c_bus.h"
#include "w4_runtime.h"
#include "mpu6500.h"
#include <math.h>
#include <string.h>

#define CAL_SAMPLES 200U
#define IO_MS 8U
#define PI 3.14159265358979323846f
osSemaphoreId_t Sem_ImuTick;
W4ImuCounters W4Imu_Counters;
static W4ImuSnapshot published;
static uint8_t raw[14], address;
static float bias_sum, gz_bias, heading;
static uint32_t prior_ms;
static bool prior_valid, calibrated;

osStatus_t W4ImuSync_Init(void)
{
    memset(&W4Imu_Counters,0,sizeof W4Imu_Counters);
    memset(&published,0,sizeof published);
    bias_sum=gz_bias=heading=0; /* Relative heading: world offset belongs to pose. */
    prior_valid=calibrated=false;
    Sem_ImuTick=osSemaphoreNew(1,0,NULL);
    return Sem_ImuTick?osOK:osErrorNoMemory;
}
void W4Imu_OnTim5Tick(void)
{
    ++W4Imu_Counters.timer_triggers;
    if(!Sem_ImuTick)return;
    if(osSemaphoreRelease(Sem_ImuTick)==osOK)++W4Imu_Counters.sem_released;
    else ++W4Imu_Counters.sem_coalesced;
}
static void publish(float gz,uint32_t now,bool valid)
{
    uint32_t mask=__get_PRIMASK();__disable_irq();
    published=(W4ImuSnapshot){heading,gz,now,valid,calibrated};
    __DMB();__set_PRIMASK(mask);
}
static bool configure(void)
{
    uint8_t id=0;
    address=0x68;
    if(!W4I2c_MemRead(address,0x75,&id,1,IO_MS)||id!=0x70){
        address=0x69;id=0;
        if(!W4I2c_MemRead(address,0x75,&id,1,IO_MS)||id!=0x70)return false;
    }
    uint8_t zero=0;
    if(!W4I2c_MemWrite(address,0x6b,&zero,1,IO_MS))return false;
    (void)osDelay(100); /* MPU wake-up settling; task context, mutex released. */
    const uint8_t regs[]={0x1b,0x1c,0x1a,0x19},values[]={0,0,3,9};
    for(unsigned j=0;j<sizeof regs;++j)
        if(!W4I2c_MemWrite(address,regs[j],&values[j],1,IO_MS))return false;
    return true;
}
void W4ImuTask(void *argument)
{
    (void)argument;
    bool configured=false;unsigned failures=0;
    for(;;){
        if(!configured){
            if(!configure()){
                ++W4Imu_Counters.init_failures;
                publish(0,HAL_GetTick(),false);
                (void)W4I2c_Recover(100);
                (void)osDelay(500);
                continue;
            }
            /* Device reconfiguration must not silently reset the world heading.
               Bias/history restart; RESET explicitly realigns the pose frame. */
            bias_sum=gz_bias=0;prior_valid=calibrated=false;
            W4Imu_Counters.calibration_samples=0;
            configured=true;failures=0;
        }
        if(osSemaphoreAcquire(Sem_ImuTick,osWaitForever)!=osOK)continue;
        uint32_t start=osKernelGetTickCount();
        W4Runtime_Begin(W4_TASK_IMU,start);
        ++W4Imu_Counters.acquired;
        bool ok=W4I2c_MemRead(address,0x3b,raw,sizeof raw,IO_MS);
        uint32_t now=HAL_GetTick();
        if(!ok){
            ++W4Imu_Counters.failed;publish(0,now,false);
            if(++failures>=3){configured=false;prior_valid=false;}
        }else{
            failures=0;
            Mpu6500Sample s;Mpu6500_Decode(raw,&s);
            ++W4Imu_Counters.completed;
            if(!calibrated){
                if(fabsf(s.ax_g)>0.25f||fabsf(s.ay_g)>0.25f||fabsf(s.az_g)<0.70f||fabsf(s.az_g)>1.30f||
                   fabsf(s.gx_dps)>5||fabsf(s.gy_dps)>5||fabsf(s.gz_dps)>5){
                    ++W4Imu_Counters.calibration_motion;W4Imu_Counters.calibration_samples=0;bias_sum=0;
                }else{
                    bias_sum+=s.gz_dps;
                    if(++W4Imu_Counters.calibration_samples>=CAL_SAMPLES){
                        gz_bias=bias_sum/CAL_SAMPLES;calibrated=true;prior_valid=false;
                    }
                }
                publish(0,now,calibrated);
            }else{
                float gz=(s.gz_dps-gz_bias)*(PI/180.0f);
                if(prior_valid){
                    uint32_t dt=now-prior_ms;
                    if(dt>0&&dt<=100){
                        heading+=gz*dt*0.001f;
                        while(heading>=PI)heading-=2*PI;
                        while(heading< -PI)heading+=2*PI;
                        if(dt>W4Imu_Counters.max_sample_interval_ms)W4Imu_Counters.max_sample_interval_ms=dt;
                    }else ++W4Imu_Counters.rejected_intervals;
                }
                prior_ms=now;prior_valid=true;publish(gz,now,true);
            }
        }
        uint32_t elapsed=osKernelGetTickCount()-start;
        if(elapsed>W4Imu_Counters.max_process_ms)W4Imu_Counters.max_process_ms=elapsed;
        W4Runtime_End(W4_TASK_IMU,10);
    }
}
uint8_t W4Imu_GetSnapshotRaw(W4ImuSnapshot *out)
{
    if(!out)return 0;
    uint32_t mask=__get_PRIMASK();__disable_irq();*out=published;__DMB();__set_PRIMASK(mask);
    return out->valid;
}
