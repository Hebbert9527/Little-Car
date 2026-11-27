#ifndef W4_I2C_BUS_H
#define W4_I2C_BUS_H

#include <stdbool.h>
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

extern osMutexId_t Mtx_I2C1;

typedef struct {
    uint32_t acquisitions, lock_timeouts, total_wait_ticks, max_wait_ticks, max_hold_ticks;
} W4I2cTiming;

typedef struct {
    W4I2cTiming all, imu, oled, recovery;
    uint32_t completed, transfer_timeouts, hal_busy, start_errors;
    uint32_t transfer_errors, nacks, abort_callbacks, peripheral_resets, ignored_callbacks;
    uint32_t recoveries, recovery_failures, last_hal_error;
    uint32_t last_wait_ticks, last_hold_ticks;
    osThreadId_t owner;
    bool available;
} W4I2cStats;

/* Call once after osKernelInitialize and the initial Week3Board_InitI2c. */
void W4I2cSync_Init(void);

/* Task-only. Addresses are unshifted. The caller retains its buffer until return.
 * Timeouts include the mutex wait. Use a positive, finite budget. */
bool W4I2c_MemRead(uint8_t addr7, uint8_t reg, uint8_t *buf, uint16_t len,
                   uint32_t timeout_ms);
bool W4I2c_MemWrite(uint8_t addr7, uint8_t reg, const uint8_t *buf, uint16_t len,
                    uint32_t timeout_ms);
bool W4I2c_Write(uint8_t addr7, const uint8_t *buf, uint16_t len, uint32_t timeout_ms);
bool W4I2c_Recover(uint32_t timeout_ms);
void W4I2c_GetStats(W4I2cStats *out);

/* Forward from the shared HAL callbacks. Flags bit 16 is reserved in I2C clients. */
void W4I2c_OnMemRxComplete(I2C_HandleTypeDef *h);
void W4I2c_OnMemTxComplete(I2C_HandleTypeDef *h);
void W4I2c_OnMasterTxComplete(I2C_HandleTypeDef *h);
void W4I2c_OnError(I2C_HandleTypeDef *h);
void W4I2c_OnAbortComplete(I2C_HandleTypeDef *h);

#endif
