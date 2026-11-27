/* Adapted from teacher W4.1 TIM6 timebase; HAL tick is independent of SysTick.
 * Original TIM6 timebase copyright (c) 2020 STMicroelectronics.
 * Original licensed under ST Ultimate Liberty license SLA0044:
 * https://www.st.com/SLA0044
 */
#include "stm32f4xx_hal.h"
#include "w4_runtime.h"
static TIM_HandleTypeDef hal_timer;
HAL_StatusTypeDef HAL_InitTick(uint32_t priority)
{
    if (priority>=(1U<<__NVIC_PRIO_BITS)) return HAL_ERROR;
    __HAL_RCC_TIM6_CLK_ENABLE();
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
    TIM6->CR1=0; TIM6->DIER=0;
    RCC_ClkInitTypeDef clocks; uint32_t latency;
    HAL_RCC_GetClockConfig(&clocks,&latency);
    uint32_t hz=HAL_RCC_GetPCLK1Freq();
    /* TIMPRE=1: timers get HCLK at APB div 1/2/4, otherwise 4*PCLK. */
    if (RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE) {
        hz=(clocks.APB1CLKDivider==RCC_HCLK_DIV1 || clocks.APB1CLKDivider==RCC_HCLK_DIV2 ||
            clocks.APB1CLKDivider==RCC_HCLK_DIV4)?HAL_RCC_GetHCLKFreq():4U*hz;
    } else if (clocks.APB1CLKDivider!=RCC_HCLK_DIV1) hz*=2;
    if (hz<1000000U || hz%1000000U) return HAL_ERROR;
    hal_timer.Instance=TIM6;
    hal_timer.Init=(TIM_Base_InitTypeDef){.Prescaler=hz/1000000U-1U,.Period=999,
        .CounterMode=TIM_COUNTERMODE_UP,.ClockDivision=TIM_CLOCKDIVISION_DIV1,
        .AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE};
    if (HAL_TIM_Base_Init(&hal_timer)!=HAL_OK) return HAL_ERROR;
    __HAL_TIM_CLEAR_FLAG(&hal_timer,TIM_FLAG_UPDATE);
    HAL_NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn,priority,0);
    uwTickPrio=priority;
    if (HAL_TIM_Base_Start_IT(&hal_timer)!=HAL_OK) return HAL_ERROR;
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    return HAL_OK;
}
void HAL_SuspendTick(void) { __HAL_TIM_DISABLE_IT(&hal_timer,TIM_IT_UPDATE); }
void HAL_ResumeTick(void) { __HAL_TIM_ENABLE_IT(&hal_timer,TIM_IT_UPDATE); }
/* TIM6 is reserved exclusively for HAL time. No shared HAL callback required. */
void TIM6_DAC_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&hal_timer,TIM_FLAG_UPDATE) &&
        __HAL_TIM_GET_IT_SOURCE(&hal_timer,TIM_IT_UPDATE)) {
        __HAL_TIM_CLEAR_IT(&hal_timer,TIM_IT_UPDATE);
        HAL_IncTick();
        W4Clock_OnHalTick();
    }
}
