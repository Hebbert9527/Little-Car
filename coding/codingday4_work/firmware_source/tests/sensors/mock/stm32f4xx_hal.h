#ifndef TEST_STM32F4XX_HAL_H
#define TEST_STM32F4XX_HAL_H

#include <stddef.h>
#include <stdint.h>

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
typedef enum { RESET, SET } FlagStatus;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef enum { I2C1_EV_IRQn, I2C1_ER_IRQn } IRQn_Type;
typedef struct { uint32_t CR1, CR2; } I2C_TypeDef;
typedef struct {
    I2C_TypeDef *Instance;
    uint8_t *pBuffPtr;
    uint32_t State, Mode, Lock, ErrorCode;
} I2C_HandleTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;

enum {
    GPIO_PIN_8 = 0x100, GPIO_PIN_9 = 0x200, GPIO_MODE_OUTPUT_OD = 1,
    GPIO_PULLUP = 1, GPIO_SPEED_FREQ_HIGH = 1, HAL_I2C_STATE_RESET = 0,
    HAL_I2C_STATE_READY, HAL_I2C_STATE_BUSY, HAL_I2C_MODE_NONE = 0, HAL_UNLOCKED = 0,
    I2C_FLAG_BUSY = 1, I2C_IT_EVT = 2, I2C_IT_BUF = 4, I2C_IT_ERR = 8,
    I2C_CR1_PE = 1, I2C_MEMADD_SIZE_8BIT = 1, HAL_I2C_ERROR_AF = 4
};

#define GPIOB ((void *)1)
#define __HAL_I2C_DISABLE_IT(h, bits) ((h)->Instance->CR2 &= ~(uint32_t)(bits))
#define __HAL_I2C_DISABLE(h) ((h)->Instance->CR1 &= ~(uint32_t)I2C_CR1_PE)
#define __HAL_I2C_GET_FLAG(h, flag) mock_bus_flag()
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_I2C1_FORCE_RESET() mock_reset()
#define __HAL_RCC_I2C1_RELEASE_RESET() ((void)0)
#define __DMB() ((void)0)
#define __DSB() ((void)0)
#define __ISB() ((void)0)

uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t);
uint32_t __get_IPSR(void);
void mock_reset(void);
FlagStatus mock_bus_flag(void);
uint32_t HAL_I2C_GetState(I2C_HandleTypeDef *);
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *);
HAL_StatusTypeDef HAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *, uint16_t);
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *, uint16_t, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t,
                                     uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t,
                                      uint8_t *, uint16_t);
void HAL_NVIC_DisableIRQ(IRQn_Type);
void HAL_NVIC_EnableIRQ(IRQn_Type);
void HAL_NVIC_ClearPendingIRQ(IRQn_Type);
void HAL_GPIO_WritePin(void *, uint16_t, GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(void *, uint16_t);
void HAL_GPIO_Init(void *, GPIO_InitTypeDef *);

#endif
