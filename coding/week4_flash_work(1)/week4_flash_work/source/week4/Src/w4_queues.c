#include "w4_queues.h"
#include "stm32f4xx_hal.h" /* PRIMASK and memory barrier intrinsics */

osMessageQueueId_t Q_Lidar;
osMessageQueueId_t Q_Cmd;
osMessageQueueId_t Q_Path;

/* Two extra slots cover a pending Put and a Get whose queue item has been
 * removed but whose consumer was preempted before reading its timestamp.
 * Publish the timestamp before Put can wake a higher-priority consumer. */
typedef struct {
    osMessageQueueId_t *handle;
    uint32_t *stamps, slots, write_index, read_index;
    W4QueueStats stats;
} QueueMeta;

static uint32_t lidar_stamps[W4_LIDAR_QUEUE_CAPACITY + 2U];
static uint32_t cmd_stamps[W4_CMD_QUEUE_CAPACITY + 2U];
static uint32_t path_stamps[W4_PATH_QUEUE_CAPACITY + 2U];
static QueueMeta queues[] = {
    { &Q_Lidar, lidar_stamps, W4_LIDAR_QUEUE_CAPACITY + 2U, 0, 0, {0} },
    { &Q_Cmd, cmd_stamps, W4_CMD_QUEUE_CAPACITY + 2U, 0, 0, {0} },
    { &Q_Path, path_stamps, W4_PATH_QUEUE_CAPACITY + 2U, 0, 0, {0} }
};

static uint32_t next_index(uint32_t index, uint32_t slots)
{
    return (index + 1U == slots) ? 0U : index + 1U;
}

static uint32_t irq_lock(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    return mask;
}

static void irq_unlock(uint32_t mask) { __set_PRIMASK(mask); }

bool W4Queues_Ready(void)
{
    return Q_Lidar != NULL && Q_Cmd != NULL && Q_Path != NULL;
}

void W4Queues_Init(void)
{
    static const osMessageQueueAttr_t lidar_attr = {.name = "Q_Lidar"};
    static const osMessageQueueAttr_t cmd_attr = {.name = "Q_Cmd"};
    static const osMessageQueueAttr_t path_attr = {.name = "Q_Path"};
    if (W4Queues_Ready()) return;
    Q_Lidar = osMessageQueueNew(W4_LIDAR_QUEUE_CAPACITY, sizeof(W4LidarBlock), &lidar_attr);
    Q_Cmd = osMessageQueueNew(W4_CMD_QUEUE_CAPACITY, sizeof(W4Command), &cmd_attr);
    Q_Path = osMessageQueueNew(W4_PATH_QUEUE_CAPACITY, sizeof(W4Waypoint), &path_attr);
    if (!W4Queues_Ready()) {
        if (Q_Lidar) (void)osMessageQueueDelete(Q_Lidar);
        if (Q_Cmd) (void)osMessageQueueDelete(Q_Cmd);
        if (Q_Path) (void)osMessageQueueDelete(Q_Path);
        Q_Lidar = Q_Cmd = Q_Path = NULL;
    }
}

bool W4Queue_GetStats(W4QueueKind kind, W4QueueStats *out)
{
    if ((unsigned)kind >= sizeof queues / sizeof queues[0] || out == NULL) return false;
    const uint32_t mask = irq_lock();
    *out = queues[kind].stats;
    irq_unlock(mask);
    return true;
}

static osStatus_t put(QueueMeta *q, const void *item, uint32_t timeout)
{
    if (*q->handle == NULL || item == NULL) return osErrorParameter;
    q->stamps[q->write_index] = osKernelGetTickCount();
    __DMB();
    const osStatus_t result = osMessageQueuePut(*q->handle, item, 0U, timeout);
    uint32_t depth = 0U;
    if (result == osOK) {
        q->write_index = next_index(q->write_index, q->slots);
        depth = osMessageQueueGetCount(*q->handle);
    }
    const uint32_t mask = irq_lock();
    if (result == osOK) {
        ++q->stats.put_ok;
        if (depth > q->stats.observed_high_water) q->stats.observed_high_water = depth;
    } else if (result == osErrorResource) ++q->stats.put_full;
    else if (result == osErrorTimeout) ++q->stats.put_timeout;
    else ++q->stats.put_error;
    irq_unlock(mask);
    return result;
}

static osStatus_t get(QueueMeta *q, void *item, uint32_t timeout)
{
    if (*q->handle == NULL || item == NULL) return osErrorParameter;
    const osStatus_t result = osMessageQueueGet(*q->handle, item, NULL, timeout);
    uint32_t latency = 0U;
    if (result == osOK) {
        __DMB();
        latency = osKernelGetTickCount() - q->stamps[q->read_index];
        q->read_index = next_index(q->read_index, q->slots);
    }
    const uint32_t mask = irq_lock();
    if (result == osOK) {
        ++q->stats.get_ok;
        q->stats.last_latency_ticks = latency;
        q->stats.latency_ticks_sum += latency;
        if (latency > q->stats.max_latency_ticks) q->stats.max_latency_ticks = latency;
    } else if (result == osErrorResource) ++q->stats.get_empty;
    else if (result == osErrorTimeout) ++q->stats.get_timeout;
    else ++q->stats.get_error;
    irq_unlock(mask);
    return result;
}

static osStatus_t reset(QueueMeta *q)
{
    if (*q->handle == NULL) return osErrorParameter;
    const uint32_t count = osMessageQueueGetCount(*q->handle);
    const osStatus_t result = osMessageQueueReset(*q->handle);
    if (result == osOK) {
        q->write_index = q->read_index = 0U;
        const uint32_t mask = irq_lock();
        q->stats.reset_discarded += count;
        irq_unlock(mask);
    }
    return result;
}

osStatus_t W4Queue_LidarPut(const W4LidarBlock *v) { return put(&queues[0], v, 0U); }
osStatus_t W4Queue_LidarGet(W4LidarBlock *v, uint32_t t) { return get(&queues[0], v, t); }
osStatus_t W4Queue_CmdPut(const W4Command *v, uint32_t t) { return put(&queues[1], v, t); }
osStatus_t W4Queue_CmdGet(W4Command *v, uint32_t t) { return get(&queues[1], v, t); }
osStatus_t W4Queue_PathPut(const W4Waypoint *v, uint32_t t) { return put(&queues[2], v, t); }
osStatus_t W4Queue_LidarReset(void) { return reset(&queues[0]); }

osStatus_t W4Queue_PathGet(W4Waypoint *v)
{
    const int32_t previous = osKernelLock();
    if (previous < 0) return osError;
    const osStatus_t result = get(&queues[2], v, 0U);
    (void)osKernelRestoreLock(previous);
    return result;
}

osStatus_t W4Queue_PathReset(void)
{
    const int32_t previous = osKernelLock();
    if (previous < 0) return osError;
    const osStatus_t result = reset(&queues[2]);
    (void)osKernelRestoreLock(previous);
    return result;
}
