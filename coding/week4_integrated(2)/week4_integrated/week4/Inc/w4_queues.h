#ifndef W4_QUEUES_H
#define W4_QUEUES_H

#include <stdbool.h>
#include <stdint.h>
#include "cmsis_os2.h"
#include "w4_contract.h"

/* The handles are defined exactly once in w4_queues.c. */
extern osMessageQueueId_t Q_Lidar;
extern osMessageQueueId_t Q_Cmd;
extern osMessageQueueId_t Q_Path;

enum { W4_LIDAR_QUEUE_CAPACITY = 8, W4_CMD_QUEUE_CAPACITY = 8,
       W4_PATH_QUEUE_CAPACITY = 64 };

typedef enum { W4_QUEUE_LIDAR, W4_QUEUE_CMD, W4_QUEUE_PATH } W4QueueKind;
typedef struct {
    uint32_t put_ok, put_full, put_timeout, put_error;
    uint32_t get_ok, get_empty, get_timeout, get_error;
    uint32_t reset_discarded, observed_high_water;
    uint32_t last_latency_ticks, max_latency_ticks;
    uint64_t latency_ticks_sum;
} W4QueueStats;

/* Called after osKernelInitialize and before threads/interrupts are enabled. */
void W4Queues_Init(void);
bool W4Queues_Ready(void);
bool W4Queue_GetStats(W4QueueKind kind, W4QueueStats *out);

/* Use these wrappers for every operation, so the by-value FIFO and its
 * latency bookkeeping remain in sync. Each queue has one producer and one
 * consumer. Queue priority is always zero; all timeout values are RTOS ticks. */
osStatus_t W4Queue_LidarPut(const W4LidarBlock *item); /* ISR, timeout 0 */
osStatus_t W4Queue_LidarGet(W4LidarBlock *item, uint32_t timeout);
osStatus_t W4Queue_CmdPut(const W4Command *item, uint32_t timeout);
osStatus_t W4Queue_CmdGet(W4Command *item, uint32_t timeout);
osStatus_t W4Queue_PathPut(const W4Waypoint *item, uint32_t timeout);
osStatus_t W4Queue_PathGet(W4Waypoint *item); /* task, nonblocking */

/* Only the LiDAR task may reset Q_Lidar, after disabling its producer.
 * Only the planner may reset Q_Path, after it has stopped producing. */
osStatus_t W4Queue_LidarReset(void);
osStatus_t W4Queue_PathReset(void);

#endif
