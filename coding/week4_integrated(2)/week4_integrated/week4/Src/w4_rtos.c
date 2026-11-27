#include "w4_rtos.h"
#include "w4_contract.h"
#include "w4_runtime.h"

const W4TaskSpec W4_TASKS[W4_TASK_COUNT] = {
    {W4ControlTask,     {.name="W4Control", .priority=osPriorityHigh,         .stack_size=2048}, 10},
    {W4ImuTask,         {.name="W4Imu",     .priority=osPriorityAboveNormal2, .stack_size=1536}, 10},
    {W4BluetoothRxTask, {.name="W4BleRx",   .priority=osPriorityAboveNormal1, .stack_size=1536}, 0},
    {W4LidarTask,       {.name="W4Lidar",   .priority=osPriorityAboveNormal,  .stack_size=2048}, 0},
    {W4PathTask,        {.name="W4Path",    .priority=osPriorityNormal,       .stack_size=2048}, 0},
    {W4BluetoothTxTask, {.name="W4BleTx",   .priority=osPriorityBelowNormal,  .stack_size=2048}, 50},
    {W4OledTask,        {.name="W4Oled",    .priority=osPriorityLow,          .stack_size=1536}, 50}
};
osThreadId_t W4Threads[W4_TASK_COUNT];
static bool initialized;

static bool objects_valid(void)
{
    return Q_Lidar && Q_Cmd && Q_Path && Sem_ImuTick && Mtx_I2C1 &&
        osMessageQueueGetCapacity(Q_Lidar)==8 &&
        osMessageQueueGetMsgSize(Q_Lidar)==sizeof(W4LidarBlock) &&
        osMessageQueueGetCapacity(Q_Cmd)==8 &&
        osMessageQueueGetMsgSize(Q_Cmd)==sizeof(W4Command) &&
        osMessageQueueGetCapacity(Q_Path)==64 &&
        osMessageQueueGetMsgSize(Q_Path)==sizeof(W4Waypoint) &&
        osSemaphoreGetCount(Sem_ImuTick)==0;
}

W4InitResult W4Rtos_Init(void)
{
    if (initialized || osKernelGetState()!=osKernelInactive) return W4_INIT_STATE;
    if (osKernelInitialize()!=osOK) return W4_INIT_KERNEL;
    if (osKernelGetTickFreq()!=1000) return W4_INIT_TICK;
    /* No notification-producing peripheral may start in these Init functions. */
    W4Queues_Init();
    W4ImuSync_Init();
    W4I2cSync_Init();
    if (!objects_valid()) return W4_INIT_OBJECT;
    for (unsigned i=0; i<W4_TASK_COUNT; ++i) {
        W4Threads[i]=osThreadNew(W4_TASKS[i].entry, NULL, &W4_TASKS[i].attr);
        if (!W4Threads[i]) {
            /* Kernel is not running, so no task can race cleanup. */
            while (i) { --i; (void)osThreadTerminate(W4Threads[i]); W4Threads[i]=NULL; }
            return W4_INIT_THREAD;
        }
    }
    initialized=true;
    W4Runtime_Init();
    return W4_INIT_OK;
}

_Noreturn void W4Rtos_Start(void)
{
    if (!initialized) W4Rtos_Fatal(W4_INIT_STATE,0);
    osStatus_t result=osKernelStart();
    W4Rtos_Fatal(W4_INIT_START_RETURNED,(uint32_t)result);
}

bool W4Rtos_CanNotify(void)
{
    osKernelState_t state=osKernelGetState();
    return initialized && (state==osKernelRunning || state==osKernelLocked);
}

bool W4Rtos_Notify(W4TaskId id, uint32_t flags)
{
    if ((unsigned)id>=W4_TASK_COUNT || !W4Rtos_CanNotify() || !W4Threads[id]) return false;
    return (osThreadFlagsSet(W4Threads[id],flags)&osFlagsError)==0;
}
