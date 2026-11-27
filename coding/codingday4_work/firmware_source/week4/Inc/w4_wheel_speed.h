#ifndef W4_WHEEL_SPEED_H
#define W4_WHEEL_SPEED_H
#include <stdbool.h>
#include <stdint.h>

typedef struct { float kp, ki; } W4WheelSpeedGains;
extern W4WheelSpeedGains w4_wheel_speed_gains;

typedef struct {
    uint16_t previous_count;
    uint32_t previous_ms;
    float rpm;
    float integral;
    float pwm;
    int32_t delta;
} W4WheelSpeedController;

int32_t W4WheelSpeed_CountDelta(uint16_t count, uint16_t previous);
/* Positive target and measured speed projected onto the requested direction. */
float W4WheelSpeed_CommandDuty(float target_rpm, float measured_rpm, float dt,
                               float feedforward, float *integral);

void W4WheelSpeed_Start(W4WheelSpeedController *wheel, uint16_t count, uint32_t now);
bool W4WheelSpeed_Update(W4WheelSpeedController *wheel, uint16_t count,
                        uint32_t now, float counts_per_rev, float target_rpm);
#endif
