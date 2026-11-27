/* Test-only motor pulses. Dual-wheel builds also accept a 5 s, 40% request.
 * TIM2 update interrupt forces all PWM outputs low. No navigation, BLE command,
 * motor PID, or automatic repeat runs in this build. Keep wheels suspended and
 * have a physical motor power disconnect available during an energized test.
 */
#if defined(W4_MOTOR_PULSE_DIAG)
#if !defined(W4_ENCODER_AB_DIAG) || defined(W4_CALIBRATED)
#error Motor pulse diagnostics require locked A/B diagnostics and forbid calibration
#endif

#include "stm32f4xx_hal.h"
#include "w4_motor_pulse_diag.h"
#ifdef W4_MOTOR_SPEED_DIAG
#include "w4_motor_speed_diag.h"
#endif

W4MotorPulseDiagnostic w4_motor_pulse_diag;

static void all_pwm_off(void)
{
    TIM4->CCR1 = TIM4->CCR2 = 0;
    TIM12->CCR1 = TIM12->CCR2 = 0;
    TIM4->EGR = TIM_EGR_UG;
    TIM12->EGR = TIM_EGR_UG;
}

int W4MotorPulse_Init(void)
{
    all_pwm_off();
    /* APB1 is 45 MHz with a doubled 90 MHz timer clock in this firmware. */
    if (HAL_RCC_GetPCLK1Freq() != 45000000u || TIM4->ARR != 4499u || TIM12->ARR != 4499u)
        return 0;
    TIM2->CR1 = 0;
    TIM2->DIER = 0;
    TIM2->PSC = 8999u;
    TIM2->ARR = 4999u;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0;
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    return 1;
}

void W4MotorPulse_Poll(void)
{
#ifdef W4_MOTOR_SPEED_DIAG
    W4MotorSpeedDiag_Poll();
#endif
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    uint32_t request = w4_motor_pulse_diag.request;
    if (request && w4_motor_pulse_diag.state == 1u) {
        /* Never queue or repeat a request that arrives during a pulse. */
        w4_motor_pulse_diag.request = 0;
    } else if (request) {
        w4_motor_pulse_diag.request = 0;
        all_pwm_off();
        uint32_t side = request <= 2u ? request : 0u;
        uint32_t left_percent = 30u;
        uint32_t right_percent = 30u;
        uint32_t duration_ms = 500u;
#ifdef W4_MOTOR_SPEED_DIAG
        uint32_t target_rpm = 0;
        W4MotorSpeedDiag_Stop();
#endif
#ifdef W4_MOTOR_PULSE_BOTH
        if (request == 3u) {
            side = 3u;
            left_percent = right_percent = 40u;
            duration_ms = 5000u;
        }
#endif
#ifdef W4_MOTOR_PULSE_TRIM
        /* WT = 5 s bench pulse, ST = 1.5 s straight-line check. Each byte
           is an integer duty percentage. Limit both to the tested 40%. */
        uint32_t mode = request >> 16;
        if (mode == 0x5754u || mode == 0x5354u) {
            left_percent = (request >> 8) & 0xffu;
            right_percent = request & 0xffu;
            if (left_percent >= 32u && left_percent <= 40u &&
                right_percent >= 32u && right_percent <= 40u) {
                side = 3u;
                duration_ms = mode == 0x5754u ? 5000u : 1500u;
            }
        }
#endif
#ifdef W4_MOTOR_SPEED_DIAG
        if ((request >> 16) == 0x5350u && (request & 0xff00u) == 0 &&
            (request & 0xffu) >= 60u && (request & 0xffu) <= 110u) {
            target_rpm = request & 0xffu;
            side = 3u;
            duration_ms = 5000u;
            left_percent = right_percent = 40u;
        }
#endif
        if (side == 0u) {
            w4_motor_pulse_diag.state = 3u;
        } else {
            w4_motor_pulse_diag.last_side = side;
            w4_motor_pulse_diag.left_start = TIM8->CNT & 0xffffu;
            w4_motor_pulse_diag.right_start = TIM3->CNT & 0xffffu;
            w4_motor_pulse_diag.start_ms = HAL_GetTick();
            w4_motor_pulse_diag.state = 1u;
#ifdef W4_MOTOR_SPEED_DIAG
            if (target_rpm) W4MotorSpeedDiag_Start(target_rpm);
#endif
            TIM2->CR1 = 0;
            /* 10 kHz counter. */
            TIM2->ARR = 10u * duration_ms - 1u;
            TIM2->CNT = 0;
            TIM2->EGR = TIM_EGR_UG;
            TIM2->SR = 0;
            TIM2->DIER = TIM_DIER_UIE;
            TIM2->CR1 = TIM_CR1_OPM | TIM_CR1_CEN;
            /* Existing logical positive mapping: left TIM12 CH2, right TIM4 CH1. */
            if (side & 1u) TIM12->CCR2 = 45u * left_percent;
            if (side & 2u) TIM4->CCR1 = 45u * right_percent;
            TIM4->EGR = TIM_EGR_UG;
            TIM12->EGR = TIM_EGR_UG;
        }
    }
    __DSB();
    __set_PRIMASK(key);
}

void W4MotorPulse_TimeoutISR(void)
{
    /* Save the active compare values before the stop for diagnostic evidence. */
    w4_motor_pulse_diag.left_ccr_before_stop = TIM12->CCR2;
    w4_motor_pulse_diag.right_ccr_before_stop = TIM4->CCR1;
    all_pwm_off();
#ifdef W4_MOTOR_SPEED_DIAG
    W4MotorSpeedDiag_Stop();
#endif
    TIM2->DIER = 0;
    TIM2->SR = 0;
    w4_motor_pulse_diag.left_stop = TIM8->CNT & 0xffffu;
    w4_motor_pulse_diag.right_stop = TIM3->CNT & 0xffffu;
    w4_motor_pulse_diag.stop_ms = HAL_GetTick();
    if (w4_motor_pulse_diag.state == 1u) {
        w4_motor_pulse_diag.request = 0;
        w4_motor_pulse_diag.state = 2u;
        ++w4_motor_pulse_diag.completed;
    }
}
#endif
