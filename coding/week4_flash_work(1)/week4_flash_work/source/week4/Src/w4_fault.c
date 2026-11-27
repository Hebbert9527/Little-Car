#include "w4_rtos.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"
volatile uint32_t W4FaultCode,W4FaultDetail;
_Noreturn void W4Rtos_Fatal(uint32_t code,uint32_t detail)
{
    __disable_irq();
    const uint32_t timers=RCC->APB1ENR;
    if ((timers & (RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM12EN)) &&
        (RCC->AHB1ENR & RCC_AHB1ENR_GPIOBEN)) {
        /* Motor driver inputs are PB6/PB7 and PB14/PB15. Preload low,
           then switch from timer output to GPIO even if PWM updates are masked. */
        const uint32_t modes=GPIO_MODER_MODER6 | GPIO_MODER_MODER7 |
                             GPIO_MODER_MODER14 | GPIO_MODER_MODER15;
        const uint32_t output=GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 |
                              GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0;
        GPIOB->BSRR=GPIO_BSRR_BR6 | GPIO_BSRR_BR7 | GPIO_BSRR_BR14 | GPIO_BSRR_BR15;
        GPIOB->MODER=(GPIOB->MODER & ~modes) | output;
    }
    /* Safe even before MX_TIM*_Init: never touch an unclocked timer. */
    if (timers & RCC_APB1ENR_TIM4EN) { TIM4->CCER=0; TIM4->CCR1=0; TIM4->CCR2=0; }
    if (timers & RCC_APB1ENR_TIM12EN) { TIM12->CCER=0; TIM12->CCR1=0; TIM12->CCR2=0; }
    W4FaultCode=code; W4FaultDetail=detail;
    __DSB();
    for (;;) __NOP();
}
void vApplicationMallocFailedHook(void) { W4Rtos_Fatal(0x100,0); }
void vApplicationStackOverflowHook(TaskHandle_t task,char *name)
{
    (void)name; W4Rtos_Fatal(0x101,(uint32_t)(uintptr_t)task);
}
void W4AssertFailed(const char *file,int line)
{
    (void)file; W4Rtos_Fatal(0x102,(uint32_t)line);
}
