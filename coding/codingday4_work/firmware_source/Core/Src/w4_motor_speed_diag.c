#if defined(W4_MOTOR_SPEED_DIAG)
#include "stm32f4xx_hal.h"
#include "w4_motor_pulse_diag.h"
#include "w4_motor_speed_diag.h"
#include "w4_wheel_speed.h"
#include <math.h>
#include <string.h>

volatile W4MotorSpeedDiagnostic w4_motor_speed_diag;
static W4WheelSpeedController left_wheel, right_wheel;
static uint32_t started_ms, previous_ms, stalled_ms[2];
static bool left_has_started;
static int32_t left_travel_counts, right_travel_counts;
static const float left_counts_per_rev = 1559.45f;
static const float right_counts_per_rev = 1561.60f;
static const uint32_t no_feedback_timeout_ms = 1000u;

void W4MotorSpeedDiag_Start(uint32_t target_rpm)
{
    memset((void *)&w4_motor_speed_diag, 0, sizeof(w4_motor_speed_diag));
    w4_motor_speed_diag.version = 1;
    w4_motor_speed_diag.target_rpm = (float)target_rpm;
    started_ms = previous_ms = HAL_GetTick();
    stalled_ms[0] = stalled_ms[1] = 0;
    left_has_started = false;
    left_travel_counts = right_travel_counts = 0;
    W4WheelSpeed_Start(&left_wheel, (uint16_t)TIM8->CNT, started_ms);
    W4WheelSpeed_Start(&right_wheel, (uint16_t)TIM3->CNT, started_ms);
    w4_motor_speed_diag.active = 1;
}

void W4MotorSpeedDiag_Stop(void)
{
    w4_motor_speed_diag.active = 0;
}

void W4MotorSpeedDiag_Poll(void)
{
    uint32_t now = HAL_GetTick();
    if (!w4_motor_speed_diag.active || now - previous_ms < 50u) return;
    uint32_t elapsed = now - previous_ms;
    previous_ms = now;
    /* Correct accumulated travel as well as instantaneous wheel speeds. */
    float turn_error = left_travel_counts / left_counts_per_rev -
                       right_travel_counts / right_counts_per_rev;
    float trim_rpm = fminf(15.0f, fmaxf(-15.0f, 45.0f * turn_error));
    float target_rpm = w4_motor_speed_diag.target_rpm;
    bool left_ok = W4WheelSpeed_Update(&left_wheel, (uint16_t)TIM8->CNT, now,
                                      left_counts_per_rev, fminf(110.0f, target_rpm - trim_rpm));
    bool right_ok = W4WheelSpeed_Update(&right_wheel, (uint16_t)TIM3->CNT, now,
                                       right_counts_per_rev, fminf(110.0f, target_rpm + trim_rpm));
    left_travel_counts += left_wheel.delta;
    right_travel_counts += right_wheel.delta;
    const bool left_is_moving = left_wheel.rpm >= 5.0f;
    left_has_started = left_has_started || left_is_moving;
    stalled_ms[0] = left_is_moving ? 0 : stalled_ms[0] + elapsed;
    stalled_ms[1] = right_wheel.rpm < 5.0f ? stalled_ms[1] + elapsed : 0;
    uint32_t fault = (!left_ok || !right_ok) ? 1u : 0u;
    if (stalled_ms[0] >= no_feedback_timeout_ms || stalled_ms[1] >= no_feedback_timeout_ms) fault = 2u;
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    /* TIM2 may have stopped the pulse while the PI computation was running. */
    if (w4_motor_speed_diag.active && w4_motor_pulse_diag.state == 1u) {
        if (fault) {
            w4_motor_speed_diag.fault = fault;
            TIM2->CR1 &= ~TIM_CR1_CEN;
            W4MotorPulse_TimeoutISR();
        } else {
            /* Boost only left-wheel breakaway, then hand control back to the PI loop. */
            float left_pwm = left_wheel.pwm;
            uint32_t since_start = now - started_ms;
            if (!left_has_started && since_start >= 150u && since_start < no_feedback_timeout_ms) {
                left_pwm = 0.60f;
            }
            TIM12->CCR2 = (uint32_t)(4500.0f * left_pwm + 0.5f);
            TIM4->CCR1 = (uint32_t)(4500.0f * right_wheel.pwm + 0.5f);
            /* Existing PWM preload latches both changes on their next cycles. */
            uint32_t index = w4_motor_speed_diag.samples;
            if (index < W4_SPEED_TRACE_CAPACITY) {
                W4SpeedTraceSample sample = {now - started_ms, left_wheel.delta, right_wheel.delta,
                    left_wheel.rpm, right_wheel.rpm, left_pwm, right_wheel.pwm};
                w4_motor_speed_diag.trace[index] = sample;
                w4_motor_speed_diag.samples = index + 1u;
            }
        }
    }
    __DSB();
    __set_PRIMASK(key);
}
#endif
