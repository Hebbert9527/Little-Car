#ifndef TEST_WEEK3_BOARD_H
#define TEST_WEEK3_BOARD_H
#include "stm32f4xx_hal.h"
extern I2C_HandleTypeDef hi2c1;
HAL_StatusTypeDef Week3Board_InitI2c(void);
#endif
