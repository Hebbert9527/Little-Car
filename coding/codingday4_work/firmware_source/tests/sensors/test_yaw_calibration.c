#include "w4_yaw_calibration.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
static Mpu6500Sample stationary={0.02f,-0.05f,1.03f,5.7f,14.6f,-0.94f};
static W4YawCalibration c;
static uint32_t now;
static W4YawCalResult push(Mpu6500Sample s){now+=10;return W4YawCal_Push(&c,&s,now);}
static void reset(void){W4YawCal_Reset(&c);now=0;}
int main(void)
{
 reset();
 for(unsigned j=0;j<199;++j)assert(push(stationary)==W4_YAW_PENDING);
 assert(c.samples==199&&push(stationary)==W4_YAW_READY);
 assert(fabsf(c.bias_dps+0.94f)<1e-5f&&c.xy_bias_warning);
 /* Measured static bias must converge without accepting noisy windows. */
 reset();
 Mpu6500Sample biased=stationary;biased.gz_dps=-6.92f;
 for(unsigned j=0;j<199;++j)assert(push(biased)==W4_YAW_PENDING);
 assert(push(biased)==W4_YAW_READY&&fabsf(c.bias_dps+6.92f)<1e-5f);
 /* A gap or I/O failure must not stitch two partial windows into a calibration. */
 reset();for(unsigned j=0;j<100;++j)push(stationary);
 now+=100;assert(push(stationary)==W4_YAW_REJECTED&&c.reason==W4_YAW_GAP&&c.samples==0);
 assert(push(stationary)==W4_YAW_PENDING&&c.samples==1);
 W4YawCal_Reset(&c);assert(!c.samples&&!c.have_time);
 /* Rotation, clipping, tilt, invalid arithmetic and acceleration all reject. */
 Mpu6500Sample s=stationary;s.gz_dps=11;assert(push(s)==W4_YAW_REJECTED);
 s=stationary;s.gx_dps=250;assert(push(s)==W4_YAW_REJECTED);
 s=stationary;s.ax_g=.5f;assert(push(s)==W4_YAW_REJECTED);
 s=stationary;s.az_g=.71f;assert(push(s)==W4_YAW_REJECTED);
 s=stationary;s.gy_dps=NAN;assert(push(s)==W4_YAW_REJECTED&&c.reason==W4_YAW_INVALID);
 /* Stable-looking but noisy windows must not provide a usable heading. */
 reset();for(unsigned j=0;j<200;++j){s=stationary;s.gz_dps+=(j&1)?.4f:-.4f;push(s);}
 assert(c.samples==0&&c.reason==W4_YAW_NOISY);
 reset();for(unsigned j=0;j<200;++j){s=stationary;s.gy_dps+=(j&1)?.4f:-.4f;push(s);}
 assert(c.samples==0&&c.reason==W4_YAW_NOISY);
 reset();for(unsigned j=0;j<200;++j){s=stationary;s.az_g+=(j&1)?.03f:-.03f;push(s);}
 assert(c.samples==0&&c.reason==W4_YAW_NOISY);
 /* A transient clears the window; a fresh full stationary window recovers. */
 reset();for(unsigned j=0;j<100;++j)push(stationary);
 s=stationary;s.gz_dps=3;assert(push(s)==W4_YAW_REJECTED&&c.samples==0);
 for(unsigned j=0;j<199;++j)assert(push(stationary)==W4_YAW_PENDING);
 assert(push(stationary)==W4_YAW_READY);
 /* Unsigned millisecond rollover and normal low-bias sensors remain supported. */
 reset();now=UINT32_MAX-1000;s=stationary;s.gx_dps=s.gy_dps=0;
 for(unsigned j=0;j<200;++j)push(s);
 assert(c.samples==200&&!c.xy_bias_warning&&fabsf(c.bias_dps+.94f)<1e-5f);
 return 0;
}
