#include "w4_imu_rtos.h"
#include "w4_i2c_bus.h"
#include "w4_runtime.h"
#include "mpu6500.h"
#include "w4_yaw_calibration.h"
#include <math.h>
#include <string.h>

#define IO_MS 8U
#define PI 3.14159265358979323846f
osSemaphoreId_t Sem_ImuTick;
W4ImuCounters W4Imu_Counters;
static W4ImuSnapshot published;
static uint8_t raw[14], address;
static float gz_bias, heading;
W4YawCalibration W4ImuCalibration;
static uint32_t prior_ms;
static bool prior_valid, calibrated;
#ifdef W4_MAZE
static bool have_bias;
#endif

#ifdef W4_IMU_DIAGNOSTICS
#include "w4_imu_diag.h"
volatile W4ImuDiagnostic W4ImuDiag;
static W4ImuDiagnostic diag_work;
static void diag_begin(void)
{
    uint32_t configurations=W4ImuDiag.configurations+1;
    memset((void *)&W4ImuDiag,0,sizeof W4ImuDiag);
    memset(&diag_work,0,sizeof diag_work);
    W4ImuDiag.magic=0x494d5531U;
    W4ImuDiag.configurations=configurations;
}
static void diag_registers(bool after)
{
    uint8_t block[16]={0};
    uint32_t mask=0;
    if(W4I2c_MemRead(address,0x13,block,10,IO_MS))mask|=1;
    if(W4I2c_MemRead(address,0x6b,block+10,2,IO_MS))mask|=2;
    if(W4I2c_MemRead(address,0x75,block+12,1,IO_MS))mask|=4;
    memcpy((void *)(after?W4ImuDiag.after:W4ImuDiag.before),block,sizeof block);
    W4ImuDiag.read_mask|=after?mask<<3:mask;
}
static void diag_sample(const Mpu6500Sample *s,uint32_t now)
{
    float values[7]={s->ax_g,s->ay_g,s->az_g,s->gx_dps,s->gy_dps,s->gz_dps,
        (float)(int16_t)(((uint16_t)raw[6]<<8)|raw[7])};
    uint32_t count=diag_work.samples+1;
    if(count==1)diag_work.first_ms=now;
    for(unsigned j=0;j<7;++j){
        if(count==1)diag_work.minimum[j]=diag_work.maximum[j]=values[j];
        float delta=values[j]-diag_work.mean[j];
        diag_work.mean[j]+=delta/(float)count;
        diag_work.m2[j]+=delta*(values[j]-diag_work.mean[j]);
        if(values[j]<diag_work.minimum[j])diag_work.minimum[j]=values[j];
        if(values[j]>diag_work.maximum[j])diag_work.maximum[j]=values[j];
    }
    diag_work.last_ms=now;
    diag_work.samples=count;
    if(count>=W4_IMU_DIAG_SAMPLES){
        /* Publish one completed window every five seconds. Readers accept only
           an unchanged, even ready sequence surrounding the memory read. */
        ++W4ImuDiag.ready;__DMB();
        memcpy((void *)W4ImuDiag.mean,diag_work.mean,sizeof diag_work.mean);
        memcpy((void *)W4ImuDiag.m2,diag_work.m2,sizeof diag_work.m2);
        memcpy((void *)W4ImuDiag.minimum,diag_work.minimum,sizeof diag_work.minimum);
        memcpy((void *)W4ImuDiag.maximum,diag_work.maximum,sizeof diag_work.maximum);
        W4ImuDiag.samples=count;
        W4ImuDiag.first_ms=diag_work.first_ms;W4ImuDiag.last_ms=now;
        __DMB();++W4ImuDiag.ready;
        memset(&diag_work,0,sizeof diag_work);
    }
}
#endif

osStatus_t W4ImuSync_Init(void)
{
    memset(&W4Imu_Counters,0,sizeof W4Imu_Counters);
    memset(&published,0,sizeof published);
    gz_bias=heading=0; /* Relative heading: world offset belongs to pose. */
    W4YawCal_Reset(&W4ImuCalibration);
    prior_valid=calibrated=false;
#ifdef W4_MAZE
    have_bias=false;
#endif
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
    published=(W4ImuSnapshot){heading,gz,now,valid,calibrated,W4ImuCalibration.xy_bias_warning};
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
#ifdef W4_IMU_DIAGNOSTICS
    diag_begin();
    diag_registers(false);
#endif
    uint8_t zero=0;
    if(!W4I2c_MemWrite(address,0x6b,&zero,1,IO_MS))return false;
    (void)osDelay(100); /* MPU wake-up settling; task context, mutex released. */
    const uint8_t regs[]={0x1b,0x1c,0x1a,0x19},values[]={0,0,3,9};
    for(unsigned j=0;j<sizeof regs;++j)
        if(!W4I2c_MemWrite(address,regs[j],&values[j],1,IO_MS))return false;
#ifdef W4_IMU_DIAGNOSTICS
    diag_registers(true);
#endif
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
               In MAZE builds, preserve the already-calibrated Z bias across
               transient I2C recoveries so motion is not interrupted waiting for
               a new 200-sample stationary calibration window. */
#ifdef W4_MAZE
            if(have_bias){
                prior_valid=false;
                calibrated=true;
            }else{
                gz_bias=0;prior_valid=calibrated=false;
                W4YawCal_Reset(&W4ImuCalibration);
                W4Imu_Counters.calibration_samples=0;
            }
#else
            gz_bias=0;prior_valid=calibrated=false;
            W4YawCal_Reset(&W4ImuCalibration);
            W4Imu_Counters.calibration_samples=0;
#endif
            configured=true;failures=0;
        }
        if(osSemaphoreAcquire(Sem_ImuTick,osWaitForever)!=osOK)continue;
        uint32_t start=osKernelGetTickCount();
        W4Runtime_Begin(W4_TASK_IMU,start);
        ++W4Imu_Counters.acquired;
        bool ok=W4I2c_MemRead(address,0x3b,raw,sizeof raw,IO_MS);
        uint32_t now=HAL_GetTick();
        if(!ok){
            ++W4Imu_Counters.failed;
            ++failures;
            /* A single failed transfer leaves the last calibrated sample in
               place, with its original timestamp and normal freshness limit. */
#ifdef W4_MAZE
            if(!calibrated||failures>=5)publish(0,now,false);
            if(!calibrated){W4YawCal_Reset(&W4ImuCalibration);W4Imu_Counters.calibration_samples=0;}
            if(failures>=5){configured=false;prior_valid=false;}
#else
            if(!calibrated||failures>=2)publish(0,now,false);
            if(!calibrated){W4YawCal_Reset(&W4ImuCalibration);W4Imu_Counters.calibration_samples=0;}
            if(failures>=3){configured=false;prior_valid=false;}
#endif
        }else{
            failures=0;
            Mpu6500Sample s;Mpu6500_Decode(raw,&s);
            ++W4Imu_Counters.completed;
#ifdef W4_IMU_DIAGNOSTICS
            diag_sample(&s,now);
#endif
            if(!calibrated){
                W4YawCalResult result=W4YawCal_Push(&W4ImuCalibration,&s,now);
                W4Imu_Counters.calibration_samples=W4ImuCalibration.samples;
                if(result==W4_YAW_REJECTED)++W4Imu_Counters.calibration_motion;
                if(result==W4_YAW_READY){
                    gz_bias=W4ImuCalibration.bias_dps;calibrated=true;prior_valid=false;
#ifdef W4_MAZE
                    have_bias=true;
#endif
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
