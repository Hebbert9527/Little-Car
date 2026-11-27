#ifndef W4_BOARD_HOOKS_H
#define W4_BOARD_HOOKS_H
#include "main.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_uart.h"
void W4Board_EnableTiming(void);
void W4Bt_OnRxComplete(UART_HandleTypeDef *h);
void W4Bt_OnUartError(UART_HandleTypeDef *h);
#endif
