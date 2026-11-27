#ifndef LIDAR_DMA_H
#define LIDAR_DMA_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "w4_contract.h"

#define LIDAR_DMA_HALF_SIZE 1250U
#define LIDAR_DMA_SIZE (2U * LIDAR_DMA_HALF_SIZE)

typedef enum
{
    LIDAR_DMA_IDLE,
    LIDAR_DMA_SEND_SYNC,
    LIDAR_DMA_WAIT_SYNC_TX,
    LIDAR_DMA_WAIT_QUIET,
    LIDAR_DMA_SEND_SCAN,
    LIDAR_DMA_WAIT_DESCRIPTOR,
    LIDAR_DMA_STREAM,
    LIDAR_DMA_WAIT_ABORT,
    LIDAR_DMA_RETRY_DELAY,
    LIDAR_DMA_FAILED
} LidarDma_State;

typedef struct
{
    LidarDma_State state;
    uint32_t ht_count;
    uint32_t tc_count;
    uint32_t last_half_interval_ms;
    uint32_t min_half_interval_ms;
    uint32_t halves_delivered;
    uint32_t bytes_delivered;
    uint32_t max_batch_ms;
    uint32_t max_copy_ms;
    uint32_t max_queue_age_ms;
    uint32_t overruns;
    uint32_t bad_descriptors;
    uint32_t descriptor_timeouts;
    uint32_t stream_timeouts;
    uint32_t hal_busy;
    uint32_t hal_failures;
    uint32_t uart_errors;
    uint32_t last_uart_error;
    uint32_t rx_stops;
    uint32_t abort_timeouts;
    uint32_t retries;
    uint32_t stream_breaks;
    uint32_t dropped_blocks;
    uint32_t sequence_gaps;
    uint32_t queue_errors;
} LidarDma_Stats;

/* Call once in the main context after initializing UART and RX DMA. */
void LidarDma_Init(UART_HandleTypeDef *huart);
/* Queues startup. Poll advances the handshake and recovery. */
bool LidarDma_Start(void);
void LidarDma_OnHalf(UART_HandleTypeDef *huart);
void LidarDma_OnFull(UART_HandleTypeDef *huart);
void LidarDma_OnError(UART_HandleTypeDef *huart);
void LidarDma_Poll(void);
/* Called only by the LiDAR task after a successful Q_Lidar Get. */
void LidarDma_ProcessBlock(const W4LidarBlock *block);

/* Live diagnostics. Individual fields are atomic on Cortex-M4. */
const volatile LidarDma_Stats *LidarDma_GetStats(void);

#endif
