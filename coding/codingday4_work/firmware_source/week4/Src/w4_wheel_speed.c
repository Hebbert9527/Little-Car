#include "w4_wheel_speed.h"
#include <math.h>
#include <string.h>

enum { W4_WHEEL_PI_MAX_RPM = 180, W4_WHEEL_BENCH_MAX_RPM = 110 };

W4WheelSpeedGains w4_wheel_speed_gains = {0.0020f, 0.0080f};

int32_t W4WheelSpeed_CountDelta(uint16_t count, uint16_t previous)
{
    uint32_t difference = (uint16_t)(count - previous);
    return difference <= 32767u ? (int32_t)difference : (int32_t)difference - 65536;
}

void W4WheelSpeed_Start(W4WheelSpeedController *wheel, uint16_t count, uint32_t now)
{
    memset(wheel, 0, sizeof(*wheel));
    wheel->previous_count = count;
    wheel->previous_ms = now;
}

static bool valid_gains(W4WheelSpeedGains gains)
{
    return isfinite(gains.kp) && isfinite(gains.ki) &&
           gains.kp >= 0 && gains.kp <= 0.01f && gains.ki >= 0 && gains.ki <= 0.05f;
}

float W4WheelSpeed_CommandDuty(float target_rpm, float measured_rpm, float dt,
                               float feedforward, float *integral)
{
    W4WheelSpeedGains gains = w4_wheel_speed_gains;
    if (!isfinite(target_rpm) || target_rpm <= 0 || target_rpm > W4_WHEEL_PI_MAX_RPM ||
        !isfinite(measured_rpm) || !isfinite(dt) || dt <= 0 ||
        !isfinite(feedforward) || !valid_gains(gains)) {
        *integral = 0;
        return 0;
    }

    float error = target_rpm - measured_rpm;
    float bias = feedforward + gains.kp * error;
    float candidate = *integral + gains.ki * error * dt;
    float candidate_duty = bias + candidate;
    /* Integrate only when unsaturated or when the error moves out of saturation. */
    if ((candidate_duty >= 0 && candidate_duty <= 0.40f) ||
        (candidate_duty > 0.40f && error < 0) || (candidate_duty < 0 && error > 0)) {
        *integral = candidate;
    }
    return fminf(0.40f, fmaxf(0, bias + *integral));
}

bool W4WheelSpeed_Update(W4WheelSpeedController *wheel, uint16_t count,
                        uint32_t now, float counts_per_rev, float target_rpm)
{
    uint32_t elapsed = now - wheel->previous_ms;
    int32_t delta = W4WheelSpeed_CountDelta(count, wheel->previous_count);
    wheel->previous_count = count;
    wheel->previous_ms = now;
    wheel->delta = delta;
    if (elapsed < 10u || elapsed > 150u || !isfinite(counts_per_rev) || counts_per_rev <= 0 ||
        !isfinite(target_rpm) || target_rpm < 0 || target_rpm > W4_WHEEL_BENCH_MAX_RPM ||
        !valid_gains(w4_wheel_speed_gains)) {
        wheel->pwm = 0;
        return false;
    }
    float dt = elapsed * 0.001f;
    float measured = delta * 60.0f / (counts_per_rev * dt);
    /* Timer polarity is verified by a forward bench pulse before navigation. */
    if (measured < -5.0f || measured > 300.0f) {
        wheel->pwm = 0;
        return false;
    }
    wheel->rpm += dt / (0.1f + dt) * (measured - wheel->rpm);
    float feedforward = 0.30f + 0.001f * target_rpm;
    wheel->pwm = W4WheelSpeed_CommandDuty(target_rpm, wheel->rpm, dt, feedforward, &wheel->integral);
    return true;
}
