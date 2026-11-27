/* Dual A/B hardware-counter initialization for navigation and diagnostics.
 * Right: E1B PA6/TIM3_CH1, E1A PA7/TIM3_CH2.
 * Left:  E2A PC6/TIM8_CH1, E2B PC7/TIM8_CH2.
 * Counters are unsigned 16-bit registers, incrementing or decrementing in x4
 * encoder mode. Their polarity and counts/revolution require physical tests.
 * No interrupt, motor output, odometry or calibration flag is enabled here.
 */
#if defined(W4_ENCODER_AB_DIAG) || defined(W4_DUAL_ENCODER)
#if defined(W4_ENCODER_AB_DIAG) && defined(W4_CALIBRATED)
#error The encoder diagnostic build must not enable motion calibration
#endif

#include "stm32f4xx_hal.h"
#include "w4_encoder_ab_diag.h"

extern TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

static HAL_StatusTypeDef configure(TIM_HandleTypeDef *timer, TIM_TypeDef *instance)
{
    TIM_Encoder_InitTypeDef encoder = {0};
    timer->Instance = instance;
    timer->Init.Prescaler = 0;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = 65535;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
    timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    encoder.EncoderMode = TIM_ENCODERMODE_TI12;
    encoder.IC1Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
    encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder.IC1Filter = 15;
    encoder.IC2Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
    encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder.IC2Filter = 15;
    return HAL_TIM_Encoder_Init(timer, &encoder);
}

int W4EncoderAbDiag_Start(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOC, &gpio);

    if (configure(&htim3, TIM3) != HAL_OK || configure(&htim8, TIM8) != HAL_OK)
        return 0;
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim8, 0);
    __HAL_DBGMCU_FREEZE_TIM3();
    __HAL_DBGMCU_FREEZE_TIM8();
    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK ||
        HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL) != HAL_OK)
        return 0;
    return 1;
}
#endif
