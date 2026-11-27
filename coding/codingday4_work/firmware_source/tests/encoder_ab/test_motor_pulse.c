#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "w4_motor_pulse_diag.h"
#ifdef W4_MOTOR_SPEED_DIAG
#include "w4_motor_speed_diag.h"
#endif

TIM_TypeDef mock_tim2, mock_tim3, mock_tim4, mock_tim8, mock_tim12;
static uint32_t pclk = 45000000, tick, mask;
static unsigned nvic_enabled;
uint32_t HAL_RCC_GetPCLK1Freq(void) { return pclk; }
uint32_t HAL_GetTick(void) { return tick; }
void HAL_NVIC_SetPriority(int irq, uint32_t p, uint32_t s) { assert(irq == TIM2_IRQn && p == 0 && s == 0); }
void HAL_NVIC_EnableIRQ(int irq) { assert(irq == TIM2_IRQn); ++nvic_enabled; }
uint32_t __get_PRIMASK(void) { return mask; }
void __disable_irq(void) { mask = 1; }
void __set_PRIMASK(uint32_t key) { mask = key; }
void __DSB(void) {}

static void assert_off(void)
{
    assert(!TIM4->CCR1 && !TIM4->CCR2 && !TIM12->CCR1 && !TIM12->CCR2);
}

int main(void)
{
    TIM4->ARR = TIM12->ARR = 4499;
    TIM4->CCR1 = TIM12->CCR2 = 123;
    assert(W4MotorPulse_Init());
    assert_off();
    assert(nvic_enabled == 1 && TIM2->PSC == 8999 && TIM2->ARR == 4999);
    w4_motor_pulse_diag.request = 99;
    W4MotorPulse_Poll();
    assert(w4_motor_pulse_diag.state == 3 && w4_motor_pulse_diag.request == 0);
    assert_off();
    for (uint32_t side = 1; side <= 2; ++side) {
        tick += 1000;
        TIM8->CNT = 100; TIM3->CNT = 200;
        w4_motor_pulse_diag.request = side;
        W4MotorPulse_Poll();
        assert(!mask && w4_motor_pulse_diag.state == 1 && w4_motor_pulse_diag.request == 0);
        assert((TIM2->CR1 & (TIM_CR1_CEN | TIM_CR1_OPM)) == (TIM_CR1_CEN | TIM_CR1_OPM));
        assert(TIM2->DIER == TIM_DIER_UIE);
        assert(TIM12->CCR2 == (side == 1 ? 1350u : 0u));
        assert(TIM4->CCR1 == (side == 2 ? 1350u : 0u));
        assert(!TIM4->CCR2 && !TIM12->CCR1);
        w4_motor_pulse_diag.request = side;
        W4MotorPulse_Poll();
        assert(w4_motor_pulse_diag.request == 0); /* no queued second pulse */
        TIM8->CNT = 110; TIM3->CNT = 220; tick += 500;
        W4MotorPulse_TimeoutISR();
        assert_off();
        assert(w4_motor_pulse_diag.state == 2 && w4_motor_pulse_diag.completed == side);
        assert(w4_motor_pulse_diag.stop_ms - w4_motor_pulse_diag.start_ms == 500);
        assert(w4_motor_pulse_diag.left_stop == 110 && w4_motor_pulse_diag.right_stop == 220);
        W4MotorPulse_Poll();
        assert_off();
    }
    w4_motor_pulse_diag.request = 3;
    W4MotorPulse_Poll();
#ifdef W4_MOTOR_PULSE_BOTH
    assert(w4_motor_pulse_diag.state == 1 && w4_motor_pulse_diag.last_side == 3);
    assert(TIM2->PSC == 8999 && TIM2->ARR == 49999);
    assert(TIM12->CCR2 == 1800 && TIM4->CCR1 == 1800);
    assert(!TIM12->CCR1 && !TIM4->CCR2);
    w4_motor_pulse_diag.request = 3;
    tick += 5000;
    W4MotorPulse_TimeoutISR();
    assert(w4_motor_pulse_diag.stop_ms - w4_motor_pulse_diag.start_ms == 5000);
    assert(w4_motor_pulse_diag.left_ccr_before_stop == 1800 &&
           w4_motor_pulse_diag.right_ccr_before_stop == 1800);
    W4MotorPulse_Poll();
    assert(w4_motor_pulse_diag.state == 2 && w4_motor_pulse_diag.completed == 3);
#else
    assert(w4_motor_pulse_diag.state == 3 && w4_motor_pulse_diag.completed == 2);
#endif
    assert_off();
#ifdef W4_MOTOR_PULSE_TRIM
    w4_motor_pulse_diag.request = 0x57542829u; /* 41% rejected */
    W4MotorPulse_Poll();
    assert(w4_motor_pulse_diag.state == 3 && TIM2->DIER == 0);
    assert_off();
    w4_motor_pulse_diag.request = 0x5354201fu; /* 31% rejected */
    W4MotorPulse_Poll();
    assert(w4_motor_pulse_diag.state == 3 && TIM2->DIER == 0);
    assert_off();
    w4_motor_pulse_diag.request = 0x57542628u; /* left 38%, right 40% */
    W4MotorPulse_Poll();
    assert(w4_motor_pulse_diag.state == 1 && TIM2->ARR == 49999);
    assert(TIM12->CCR2 == 1710 && TIM4->CCR1 == 1800);
    W4MotorPulse_TimeoutISR();
    assert_off();
    w4_motor_pulse_diag.request = 0x53542826u; /* left 40%, right 38% */
    W4MotorPulse_Poll();
    assert(w4_motor_pulse_diag.state == 1 && TIM2->ARR == 14999);
    assert(TIM12->CCR2 == 1800 && TIM4->CCR1 == 1710);
    W4MotorPulse_TimeoutISR();
    assert_off();
#endif
#ifdef W4_MOTOR_SPEED_DIAG
    w4_motor_pulse_diag.request = 0x5350006fu; /* 111 RPM rejected */
    W4MotorPulse_Poll();
    assert(w4_motor_pulse_diag.state == 3);
    assert_off();
    w4_motor_pulse_diag.request = 0x5350005au; /* 90 RPM, both wheels */
    W4MotorPulse_Poll();
    assert(w4_motor_speed_diag.active && w4_motor_pulse_diag.state == 1);
    assert(w4_motor_speed_diag.target_rpm == 90 && TIM2->ARR == 49999);
    for (unsigned i = 1; i <= 3; ++i) {
        tick += 50; TIM3->CNT += 125;
        W4MotorPulse_Poll();
        assert(TIM12->CCR2 == (i < 3 ? 1800u : 2700u) && TIM4->CCR1 <= 1800u);
    }
    assert(w4_motor_speed_diag.samples == 3 && !w4_motor_speed_diag.fault);
    assert(w4_motor_speed_diag.trace[2].left_pwm == 0.60f);
    tick += 50; TIM8->CNT += 120; TIM3->CNT += 125;
    W4MotorPulse_Poll();
    assert(w4_motor_speed_diag.samples == 4 && !w4_motor_speed_diag.fault);
    for (unsigned i = 0; i < 7; ++i) {
        assert(TIM12->CCR2 <= 1800 && TIM4->CCR1 <= 1800);
        tick += 50;
        W4MotorPulse_Poll();
    }
    assert(TIM12->CCR2 <= 1800 && w4_motor_speed_diag.trace[10].left_rpm < 5.0f);
    W4MotorPulse_TimeoutISR();
    tick += 50;
    W4MotorPulse_Poll();
    assert(!w4_motor_speed_diag.active);
    assert_off();
    w4_motor_pulse_diag.request = 0x5350005au;
    W4MotorPulse_Poll();
    for (unsigned i = 1; i < 20; ++i) {
        tick += 50;
        W4MotorPulse_Poll();
        assert(TIM12->CCR2 == (i < 3 ? 1800u : 2700u) && TIM4->CCR1 == 1800u);
    }
    tick += 50;
    W4MotorPulse_Poll();
    assert(w4_motor_speed_diag.fault == 2 && !w4_motor_speed_diag.active);
    assert_off();

    /* Unequal simulated motors should recover the accumulated start-up error. */
    TIM8->CNT = TIM3->CNT = 0;
    w4_motor_pulse_diag.request = 0x5350005au;
    W4MotorPulse_Poll();
    float left_rpm = 0, right_rpm = 0;
    float left_counts = 0, right_counts = 0;
    for (unsigned i = 0; i < 100; ++i) {
        float left_rpm_at_duty = fmaxf(0.0f, 550.0f * (TIM12->CCR2 / 4500.0f - 0.12f));
        float right_rpm_at_duty = fmaxf(0.0f, 520.0f * (TIM4->CCR1 / 4500.0f - 0.151f));
        left_rpm += (left_rpm_at_duty - left_rpm) / 12.0f;
        right_rpm += (right_rpm_at_duty - right_rpm) / 13.0f;
        left_counts += left_rpm * (1559.45f / 1200.0f);
        right_counts += right_rpm * (1561.60f / 1200.0f);
        TIM8->CNT = (uint32_t)left_counts;
        TIM3->CNT = (uint32_t)right_counts;
        tick += 50;
        W4MotorPulse_Poll();
    }
    assert(w4_motor_speed_diag.samples == 100 && !w4_motor_speed_diag.fault);
    assert(fabsf(left_counts / 1559.45f - right_counts / 1561.60f) < 0.05f);
    assert(fabsf(left_rpm - 90.0f) < 3.0f && fabsf(right_rpm - 90.0f) < 3.0f);
    W4MotorPulse_TimeoutISR();
    assert_off();
#endif
    pclk = 123;
    TIM4->CCR1 = 100;
    assert(!W4MotorPulse_Init());
    assert_off();
    puts("PASS PWM pulse, stop, rejection and no repeat");
    return 0;
}
