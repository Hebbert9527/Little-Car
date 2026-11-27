#ifndef MPU6500_DECODE_H
#define MPU6500_DECODE_H
#include <stdint.h>
/* Burst starting at ACCEL_XOUT_H (0x3B), including temperature bytes. */
typedef struct
{
    float ax_g, ay_g, az_g;
    float gx_dps, gy_dps, gz_dps;
} Mpu6500Sample;
void Mpu6500_Decode(const uint8_t frame[14], Mpu6500Sample *sample);
#endif
