#include <assert.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "w4_encoder_ab_diag.h"

TIM_TypeDef mock_tim3, mock_tim8;
GPIO_TypeDef mock_gpioa, mock_gpioc;
TIM_HandleTypeDef htim3;
static unsigned gpio_calls, init_calls, start_calls, fail_init, fail_start;

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio)
{
    ++gpio_calls;
    assert(gpio->Pin == (GPIO_PIN_6 | GPIO_PIN_7));
    assert(gpio->Mode == GPIO_MODE_AF_PP && gpio->Pull == GPIO_NOPULL);
    assert((port == GPIOA && gpio->Alternate == GPIO_AF2_TIM3) ||
           (port == GPIOC && gpio->Alternate == GPIO_AF3_TIM8));
}

HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *timer, const TIM_Encoder_InitTypeDef *cfg)
{
    ++init_calls;
    assert((init_calls == 1 && timer->Instance == TIM3) ||
           (init_calls == 2 && timer->Instance == TIM8));
    assert(timer->Init.Prescaler == 0 && timer->Init.Period == 65535);
    assert(cfg->EncoderMode == TIM_ENCODERMODE_TI12);
    assert(cfg->IC1Selection == TIM_ICSELECTION_DIRECTTI &&
           cfg->IC2Selection == TIM_ICSELECTION_DIRECTTI);
    return init_calls == fail_init ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
    ++start_calls;
    assert((start_calls == 1 && timer->Instance == TIM3) ||
           (start_calls == 2 && timer->Instance == TIM8));
    assert(channel == TIM_CHANNEL_ALL);
    assert(timer->Instance->CNT == 0);
    return start_calls == fail_start ? HAL_ERROR : HAL_OK;
}

static void reset_test(void)
{
    gpio_calls = init_calls = start_calls = fail_init = fail_start = 0;
    mock_tim3.CNT = mock_tim8.CNT = 123;
}

int main(void)
{
    reset_test();
    assert(W4EncoderAbDiag_Start() == 1);
    assert(gpio_calls == 2 && init_calls == 2 && start_calls == 2);
    for (unsigned failed = 1; failed <= 2; ++failed) {
        reset_test(); fail_init = failed;
        assert(W4EncoderAbDiag_Start() == 0);
        assert(init_calls == failed && start_calls == 0);
        reset_test(); fail_start = failed;
        assert(W4EncoderAbDiag_Start() == 0);
        assert(start_calls == failed);
    }
    puts("PASS dual encoder setup and failure paths");
    return 0;
}
