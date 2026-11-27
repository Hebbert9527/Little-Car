#ifndef W4_RTOS_H
#define W4_RTOS_H
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdint.h>
typedef enum {
    W4_TASK_CONTROL, W4_TASK_IMU, W4_TASK_RX, W4_TASK_LIDAR,
    W4_TASK_PATH, W4_TASK_TX, W4_TASK_OLED, W4_TASK_COUNT
} W4TaskId;
typedef struct {
    osThreadFunc_t entry;
    osThreadAttr_t attr;
    uint32_t period_ticks; /* expected release period for stats; zero = unpaced event */
} W4TaskSpec;
typedef enum {
    W4_INIT_OK, W4_INIT_STATE, W4_INIT_KERNEL, W4_INIT_TICK,
    W4_INIT_OBJECT, W4_INIT_THREAD, W4_INIT_START_RETURNED
} W4InitResult;
extern const W4TaskSpec W4_TASKS[W4_TASK_COUNT];
extern osThreadId_t W4Threads[W4_TASK_COUNT];
/* Defined exactly once by members 3, 4, 5 respectively, never by w4_rtos.c. */
extern osMessageQueueId_t Q_Lidar, Q_Cmd, Q_Path;
extern osSemaphoreId_t Sem_ImuTick;
extern osMutexId_t Mtx_I2C1;
/* Flags are per-thread. Reserve these bits consistently with each task owner. */
#define W4_FLAG_RX_DATA       (1UL << 0)
#define W4_FLAG_TX_COMPLETE   (1UL << 0)
#define W4_FLAG_TX_MAILBOX    (1UL << 1)
#define W4_FLAG_TX_ERROR      (1UL << 2)
#define W4_FLAG_I2C_COMPLETE  (1UL << 0)
#define W4_FLAG_I2C_ERROR     (1UL << 1)
extern volatile uint32_t W4FaultCode;
extern volatile uint32_t W4FaultDetail;
W4InitResult W4Rtos_Init(void);
/* Does not return. Even osOK returned by this teacher wrapper means start failed. */
_Noreturn void W4Rtos_Start(void);
bool W4Rtos_CanNotify(void);
/* Owner calls only after the kernel is running. Keeps ISR bounded. */
bool W4Rtos_Notify(W4TaskId id, uint32_t flags);
/* Direct PWM emergency stop, no dependency on another member or the kernel. */
_Noreturn void W4Rtos_Fatal(uint32_t code, uint32_t detail);
#endif
