#ifndef WEEK3_BOARD_H
#define WEEK3_BOARD_H
#include "stm32f4xx_hal.h"
extern UART_HandleTypeDef huart1, huart3;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim5;
extern volatile uint32_t week3_init_errors;
void Week3Board_InitBluetooth(void);
void Week3Board_InitLidar(void);
HAL_StatusTypeDef Week3Board_InitI2c(void);
void Week3Board_StartImuTimer(void);
#endif
