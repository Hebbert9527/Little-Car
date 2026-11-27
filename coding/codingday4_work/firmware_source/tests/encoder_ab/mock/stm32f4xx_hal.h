#ifndef ENCODER_AB_TEST_HAL_H
#define ENCODER_AB_TEST_HAL_H
#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef struct { uint32_t CR1, DIER, PSC, ARR, EGR, SR, CNT, CCR1, CCR2; } TIM_TypeDef;
typedef struct { uint32_t unused; } GPIO_TypeDef;
typedef struct {
    uint32_t Prescaler, CounterMode, Period, ClockDivision, AutoReloadPreload;
} TIM_Base_InitTypeDef;
typedef struct { TIM_TypeDef *Instance; TIM_Base_InitTypeDef Init; } TIM_HandleTypeDef;
typedef struct {
    uint32_t EncoderMode, IC1Polarity, IC1Selection, IC1Prescaler, IC1Filter;
    uint32_t IC2Polarity, IC2Selection, IC2Prescaler, IC2Filter;
} TIM_Encoder_InitTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
extern TIM_TypeDef mock_tim2, mock_tim3, mock_tim4, mock_tim8, mock_tim12;
extern GPIO_TypeDef mock_gpioa, mock_gpioc;
#define TIM3 (&mock_tim3)
#define TIM8 (&mock_tim8)
#define TIM2 (&mock_tim2)
#define TIM4 (&mock_tim4)
#define TIM12 (&mock_tim12)
#define GPIOA (&mock_gpioa)
#define GPIOC (&mock_gpioc)
#define GPIO_PIN_6 (1u << 6)
#define GPIO_PIN_7 (1u << 7)
#define GPIO_MODE_AF_PP 2u
#define GPIO_NOPULL 0u
#define GPIO_SPEED_FREQ_LOW 0u
#define GPIO_AF2_TIM3 2u
#define GPIO_AF3_TIM8 3u
#define TIM_COUNTERMODE_UP 0u
#define TIM_CLOCKDIVISION_DIV4 4u
#define TIM_AUTORELOAD_PRELOAD_DISABLE 0u
#define TIM_ENCODERMODE_TI12 3u
#define TIM_INPUTCHANNELPOLARITY_RISING 0u
#define TIM_ICSELECTION_DIRECTTI 1u
#define TIM_ICPSC_DIV1 0u
#define TIM_CHANNEL_ALL 0xffu
#define TIM2_IRQn 28
#define TIM_EGR_UG 1u
#define TIM_DIER_UIE 1u
#define TIM_CR1_OPM 8u
#define TIM_CR1_CEN 1u
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM3_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM8_CLK_ENABLE() ((void)0)
#define __HAL_TIM_SET_COUNTER(h, n) ((h)->Instance->CNT = (n))
#define __HAL_DBGMCU_FREEZE_TIM3() ((void)0)
#define __HAL_DBGMCU_FREEZE_TIM8() ((void)0)
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio);
HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *timer, const TIM_Encoder_InitTypeDef *cfg);
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *timer, uint32_t channel);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_GetTick(void);
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t subpriority);
void HAL_NVIC_EnableIRQ(int irq);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t key);
void __DSB(void);
#endif
