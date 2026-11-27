#include "w4_runtime.h"
#include "stm32f4xx.h"
static volatile bool ready;
static uint32_t last_cycle,clock_hz;
static uint64_t extended;
void W4Clock_Init(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT=0;
    DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    last_cycle=0; extended=0; clock_hz=SystemCoreClock; ready=true;
    __set_PRIMASK(mask);
}
void W4Clock_OnHalTick(void)
{
    if (!ready) return;
    uint32_t now=DWT->CYCCNT;
    extended+=(uint32_t)(now-last_cycle); last_cycle=now;
}
uint64_t W4Clock_Cycles(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    uint64_t now=extended+(uint32_t)(DWT->CYCCNT-last_cycle);
    __set_PRIMASK(mask);
    return now;
}
uint32_t W4Clock_Hz(void) { return clock_hz; }
