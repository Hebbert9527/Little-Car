#ifndef W4_IMU_RTOS_H
#define W4_IMU_RTOS_H
#include "cmsis_os2.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define W4_IMU_SAMPLE_BYTES 14U
#define W4_IMU_THREAD_FLAG_DONE (1UL << 0)
#define W4_IMU_THREAD_FLAG_ERROR (1UL << 1)
typedef struct {
    float heading_rad, gz_rad_s;
    uint32_t sample_ms;
    uint8_t valid, calibrated;
} W4ImuSnapshot;
typedef struct {
    volatile uint32_t timer_triggers, sem_released, sem_coalesced;
    volatile uint32_t acquired, completed, failed;
    volatile uint32_t max_wait_ms, max_process_ms, rejected_intervals;
    volatile uint32_t calibration_motion, calibration_samples;
    volatile uint32_t init_failures, max_sample_interval_ms;
} W4ImuCounters;
extern osSemaphoreId_t Sem_ImuTick;
extern osMutexId_t Mtx_I2C1;
extern W4ImuCounters W4Imu_Counters;
/* Called once after RTOS objects exist; creates Sem_ImuTick(max=1, initial=0). */
osStatus_t W4ImuSync_Init(void);
/* TIM5 callback hook: invoke from ISR only. */
void W4Imu_OnTim5Tick(void);
/* Task entry: argument unused. Requires Mtx_I2C1 from member 5. */
void W4ImuTask(void *argument);
/* Coherent copy of the most recently published state.
 * Renamed from W4Imu_GetSnapshot: the frozen contract owns that name and exposes
 * a W4ImuSample adapter (see w4_integration.c). This raw getter keeps member4's
 * own W4ImuSnapshot layout. */
uint8_t W4Imu_GetSnapshotRaw(W4ImuSnapshot *out);
#ifdef __cplusplus
}
#endif
#endif


