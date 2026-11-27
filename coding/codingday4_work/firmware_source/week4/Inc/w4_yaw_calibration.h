#ifndef W4_YAW_CALIBRATION_H
#define W4_YAW_CALIBRATION_H
#include "mpu6500.h"
#include <stdint.h>
#define W4_YAW_CAL_SAMPLES 200U
typedef enum { W4_YAW_PENDING, W4_YAW_READY, W4_YAW_REJECTED } W4YawCalResult;
typedef enum { W4_YAW_NONE, W4_YAW_INVALID, W4_YAW_MOVING, W4_YAW_NOISY, W4_YAW_GAP } W4YawCalReason;
typedef struct {
    uint32_t samples, last_ms;
    float mean[6], m2[6], minimum[6], maximum[6], bias_dps;
    uint8_t have_time, xy_bias_warning, reason;
} W4YawCalibration;
void W4YawCal_Reset(W4YawCalibration *state);
/* For horizontal planar yaw only. Keep the robot physically stationary.
 * Stable X/Y bias is reported, not used as yaw. This is not 6-axis calibration
 * or certification of a sensor with out-of-spec X/Y zero-rate output. */
W4YawCalResult W4YawCal_Push(W4YawCalibration *state,const Mpu6500Sample *sample,uint32_t ms);
#endif
