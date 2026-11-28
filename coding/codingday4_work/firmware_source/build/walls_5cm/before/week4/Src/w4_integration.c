/* ============================================================================
 * w4_integration.c — Member 1 integration glue for the Week4 RTOS build.
 *
 * Build mode: W4_ENABLE_RTOS. This file is the ONLY place that defines the
 * global HAL callbacks for the RTOS firmware; the old bare-metal
 * member1..5/firmware/callbacks.c and week3.c MUST NOT be linked in this mode.
 *
 * It supplies the contract symbols no member implemented:
 *   - W4BluetoothTxTask      (50 ms telemetry publisher + USART3 TX DMA pump)
 *   - W4Telemetry_SubmitLine (thread-safe line mailbox, single TX consumer)
 *   - W4Oled_ReadNavSnapshot (coherent display data over the shared I2C1 bus)
 * plus the contract W4Imu_GetSnapshot adapter, App_OnLidarPoint (P-frame source
 * for member3's parser), the centralized HAL callback dispatch, and the RTOS
 * start entry W4Integration_Start().
 *
 * Original Week3 functionality preserved: LiDAR P point-cloud frames, IMU I
 * frames and BLE telemetry keep the same ASCII protocol; the new N navigation
 * frame is added. Week2 PWM/encoder/ADC hardware init is reused unchanged.
 * ==========================================================================*/
#include "w4_integration.h"
#include "w4_contract.h"
#include "w4_rtos.h"
#include "w4_runtime.h"
#include "w4_period.h"
#include "w4_nav.h"
#include "w4_board_hooks.h"
#include "w4_imu_rtos.h"
#include "w4_i2c_bus.h"
#include "w4_oled.h"
#include "lidar_dma.h"
#ifdef W4_MAZE
#include "w4_maze.h"
#endif
#include "week3_board.h"
#include "main.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* member3's parser calls this synchronously from W4LidarTask context. */
void App_OnLidarPoint(float angle_deg, float distance_mm, uint8_t quality, bool new_scan);

/* ---------------------------------------------------------------------------
 * Telemetry mailbox: many producers (LiDAR/RX/Path/TX tasks), one consumer
 * (W4BluetoothTxTask). A by-value CMSIS queue is the protected mailbox.
 * -------------------------------------------------------------------------*/
#define TX_LINE_MAX   264U
#define TX_MBOX_DEPTH 18U
static osMessageQueueId_t s_tx_mbox;
static uint32_t s_tx_dropped, s_tx_errors, s_tx_sent;
static uint32_t s_p_seq, s_i_seq;
static uint32_t s_last_point_ms;
static bool s_point_attempted, s_scan_pending;

bool W4Telemetry_SubmitLine(const char *line, uint16_t len)
{
    if (line == NULL || len == 0U || len >= TX_LINE_MAX || s_tx_mbox == NULL) {
        ++s_tx_dropped;
        return false;
    }
    char buf[TX_LINE_MAX]={0};
    memcpy(buf, line, len);
    buf[len] = '\0';
    if (osMessageQueuePut(s_tx_mbox, buf, 0U, 0U) != osOK) {
        ++s_tx_dropped;
        return false;
    }
    (void)W4Rtos_Notify(W4_TASK_TX, W4_FLAG_TX_MAILBOX);
    return true;
}

/* P point-cloud frame, throttled to 10 ms exactly like the Week3 app.c. */
void App_OnLidarPoint(float angle_deg, float distance_mm, uint8_t quality, bool new_scan)
{
    const uint32_t now = HAL_GetTick();
#ifdef W4_MAZE
    W4Maze_Point(angle_deg,distance_mm,quality,new_scan,now);
#endif
    if (new_scan) s_scan_pending = true;
    if (s_point_attempted && (uint32_t)(now - s_last_point_ms) < 10U) return;
    s_point_attempted = true;
    s_last_point_ms = now;
    char line[TX_LINE_MAX];
    int n = snprintf(line, sizeof line, "P,%lu,%lu,%ld,%ld,%u,%u\n",
                     (unsigned long)(s_p_seq++), (unsigned long)now,
                     (long)lroundf(angle_deg * 100.0f), (long)lroundf(distance_mm),
                     (unsigned)quality, (unsigned)(s_scan_pending ? 1U : 0U));
    if (n > 0 && n < (int)sizeof line) {
        if (W4Telemetry_SubmitLine(line, (uint16_t)n)) s_scan_pending = false;
    }
}

/* IMU snapshot adapter: contract W4ImuSample over member4's raw W4ImuSnapshot. */
bool W4Imu_GetSnapshot(W4ImuSample *out)
{
    if (out == NULL) return false;
    W4ImuSnapshot raw;
    uint8_t valid = W4Imu_GetSnapshotRaw(&raw);
    out->stamp_ms    = raw.sample_ms;
    out->heading_rad = raw.heading_rad;
    out->gz_rad_s    = raw.gz_rad_s;
    out->valid       = raw.valid;
    out->calibrated  = raw.calibrated;
    return valid != 0U;
}

/* One synchronized navigation snapshot, with display units mm/mrad. */
bool W4Oled_ReadNavSnapshot(W4OledNavSnapshot *out)
{
    W4NavSnapshot n;
    if(!out || !W4Nav_GetSnapshot(&n))return false;
    W4ImuSnapshot imu={0};W4Imu_GetSnapshotRaw(&imu);
    uint32_t now=HAL_GetTick();
    *out=(W4OledNavSnapshot){
        .x_mm=(int32_t)lroundf(n.pose.x_m*1000),.y_mm=(int32_t)lroundf(n.pose.y_m*1000),
        .theta_mrad=(int32_t)lroundf(n.pose.theta_rad*1000),
        .target_x_mm=(int32_t)lroundf(n.target_x*1000),.target_y_mm=(int32_t)lroundf(n.target_y*1000),
        .state=(W4OledNavState)n.state,.fault=n.fault,
        .pose_valid=n.pose.valid!=0&&now-n.pose.stamp_ms<=50U,.target_valid=n.cmd_id!=0,
        .imu_heading_deg=(int32_t)lroundf(imu.heading_rad*57.295779513f),
        .imu_calibration_samples=W4Imu_Counters.calibration_samples,
        .imu_valid=imu.valid&&imu.calibrated&&now-imu.sample_ms<=50U,
        .imu_xy_bias_warning=imu.xy_bias_warning!=0,.motion_locked=!W4HardwareReady()
    };
    return true;
}

/* ---------------------------------------------------------------------------
 * Centralized HAL callbacks — the single dispatch point for the RTOS firmware.
 * -------------------------------------------------------------------------*/
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *h)
{
    if (h == &huart1) LidarDma_OnHalf(h);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)
{
    if (h == &huart1)      LidarDma_OnFull(h);
    else if (h == &huart3) W4Bt_OnRxComplete(h);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h)
{
    if (h == &huart3) (void)W4Rtos_Notify(W4_TASK_TX, W4_FLAG_TX_COMPLETE);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *h)
{
    if (h == &huart1) {
        LidarDma_OnError(h);
    } else if (h == &huart3) {
        W4Bt_OnUartError(h);
        (void)W4Rtos_Notify(W4_TASK_TX, W4_FLAG_TX_ERROR);
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *h)
{
    W4I2c_OnMemRxComplete(h);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *h)
{
    W4I2c_OnMemTxComplete(h);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *h)
{
    W4I2c_OnMasterTxComplete(h);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *h)
{
    W4I2c_OnError(h);
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *h)
{
    W4I2c_OnAbortComplete(h);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *h)
{
    if (h == &htim5) W4Imu_OnTim5Tick();   /* TIM6 has its own dedicated ISR */
}

/* ---------------------------------------------------------------------------
 * Bluetooth TX task: drains the mailbox over USART3 TX DMA and publishes I/N
 * frames every 50 ms. DMA completion/error wake it via thread flags.
 * -------------------------------------------------------------------------*/
static void tx_publish_periodic(uint32_t now)
{
    W4Runtime_Begin(W4_TASK_TX,osKernelGetTickCount());
    char line[TX_LINE_MAX];
    W4ImuSample imu;
    if (W4Imu_GetSnapshot(&imu) && imu.valid) {
        int n = snprintf(line, sizeof line, "I,%lu,%lu,%ld,%ld\n",
                         (unsigned long)(s_i_seq++), (unsigned long)now,
                         (long)lroundf(imu.heading_rad * 5729.5779513f),
                         (long)lroundf(imu.gz_rad_s * 5729.5779513f));
        if (n > 0 && n < (int)sizeof line)
            (void)W4Telemetry_SubmitLine(line, (uint16_t)n);
    }
    uint16_t nn = W4Nav_FormatTelemetry(line, sizeof line, now);
    if (nn) (void)W4Telemetry_SubmitLine(line, nn);
#if defined(W4_DUAL_ENCODER) && defined(W4_WHEEL_DIAGNOSTICS)
    static uint32_t last_wheels_ms;
    if (now-last_wheels_ms>=100U) {
        last_wheels_ms=now;
        uint16_t wheel_length=W4Motor_FormatDiagnostics(line,sizeof line);
        if (wheel_length) (void)W4Telemetry_SubmitLine(line,wheel_length);
    }
#endif
    W4Runtime_End(W4_TASK_TX,50);
}

/* Once per second: real bus ownership counters, sensor health, and one task.
   Rotate task details to bound UART traffic and avoid overflowing the mailbox. */
static void tx_diagnostics(uint32_t now)
{
    static unsigned task;
    W4I2cStats bus; W4OledStats oled; W4ImuSnapshot imu;
    W4I2c_GetStats(&bus); W4Oled_GetStats(&oled); W4Imu_GetSnapshotRaw(&imu);
    W4Runtime_CollectResources();
    char line[TX_LINE_MAX];
    int n=snprintf(line,sizeof line,"B,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n",
        (unsigned long)now,(unsigned long)bus.imu.acquisitions,(unsigned long)bus.oled.acquisitions,
        (unsigned long)bus.imu.max_wait_ticks,(unsigned long)bus.oled.max_hold_ticks,
        (unsigned long)bus.completed,(unsigned long)bus.transfer_timeouts,(unsigned long)bus.recoveries);
    if(n>0&&n<(int)sizeof line)(void)W4Telemetry_SubmitLine(line,(uint16_t)n);
    n=snprintf(line,sizeof line,"S,%lu,%lu,%lu,%lu,%lu,%u,%u\n",
        (unsigned long)now,(unsigned long)W4Imu_Counters.timer_triggers,
        (unsigned long)W4Imu_Counters.completed,(unsigned long)W4Imu_Counters.failed,
        (unsigned long)W4Imu_Counters.sem_coalesced,(unsigned)imu.calibrated,(unsigned)oled.status);
    if(n>0&&n<(int)sizeof line)(void)W4Telemetry_SubmitLine(line,(uint16_t)n);
    W4Stats nav_stats; W4Nav_GetStats(&nav_stats);
    n=snprintf(line,sizeof line,"Q,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n",
        (unsigned long)now,(unsigned long)osMessageQueueGetCount(Q_Lidar),
        (unsigned long)osMessageQueueGetCapacity(Q_Lidar),(unsigned long)osMessageQueueGetCount(Q_Cmd),
        (unsigned long)osMessageQueueGetCapacity(Q_Cmd),(unsigned long)osMessageQueueGetCount(Q_Path),
        (unsigned long)osMessageQueueGetCapacity(Q_Path),(unsigned long)nav_stats.cmd_full,
        (unsigned long)nav_stats.path_wait);
    if(n>0&&n<(int)sizeof line)(void)W4Telemetry_SubmitLine(line,(uint16_t)n);
    W4TaskStats t; W4Runtime_Get((W4TaskId)task,&t);
    uint32_t mhz=W4Clock_Hz()/1000000U; if(!mhz)mhz=1;
    n=snprintf(line,sizeof line,"T,%lu,%u,%s,%u,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n",
        (unsigned long)now,task,W4_TASKS[task].attr.name,(unsigned)W4_TASKS[task].attr.priority,
        (unsigned long)W4_TASKS[task].attr.stack_size,(unsigned long)t.wakes,
        (unsigned long)(t.period_cycles.max/mhz),(unsigned long)(t.elapsed_cycles.max/mhz),
        (unsigned long)(t.jitter_cycles.max/mhz),(unsigned long)t.deadline_misses,
        (unsigned long)t.stack_free_min);
    if(n>0&&n<(int)sizeof line)(void)W4Telemetry_SubmitLine(line,(uint16_t)n);
    task=(task+1U)%W4_TASK_COUNT;
}

void W4BluetoothTxTask(void *argument)
{
    (void)argument;
    uint32_t deadline = osKernelGetTickCount() + 50U;
    bool dma_busy = false;
    uint32_t last_diagnostics=osKernelGetTickCount();
    static char line[TX_LINE_MAX];   /* static: must stay valid during DMA */
    for (;;) {
        if((uint32_t)(osKernelGetTickCount()-last_diagnostics)>=1000){
            last_diagnostics=osKernelGetTickCount();
            tx_diagnostics(HAL_GetTick());
        }
        if (!dma_busy && s_tx_mbox != NULL &&
            osMessageQueueGet(s_tx_mbox, line, 0U, 0U) == osOK) {
            uint16_t n = (uint16_t)strnlen(line, TX_LINE_MAX);
            if (n && HAL_UART_Transmit_DMA(&huart3, (uint8_t *)line, n) == HAL_OK) {
                dma_busy = true;
                ++s_tx_sent;
            } else {
                ++s_tx_errors;
            }
        }
        uint32_t wait = W4Period_Remaining(deadline, osKernelGetTickCount());
        uint32_t f = osThreadFlagsWait(W4_FLAG_TX_COMPLETE | W4_FLAG_TX_MAILBOX | W4_FLAG_TX_ERROR,
                                       osFlagsWaitAny, wait);
        if (f & osFlagsError) {                 /* timeout → 50 ms publication */
            tx_publish_periodic(HAL_GetTick());
            deadline += 50U;
        } else {
            if (f & W4_FLAG_TX_COMPLETE) dma_busy = false;
            if (f & W4_FLAG_TX_ERROR) {
                dma_busy = false;
                ++s_tx_errors;
                (void)HAL_UART_AbortTransmit(&huart3);  /* TX only; keep RX IT */
            }
            if ((int32_t)(osKernelGetTickCount() - deadline) >= 0) {
                tx_publish_periodic(HAL_GetTick());
                deadline += 50U;
            }
        }
    }
}

/* ---------------------------------------------------------------------------
 * RTOS start entry. Called from main() in W4_ENABLE_RTOS mode after the shared
 * HAL/clock/MX peripheral init, replacing Week3_Init() + the bare-metal loop.
 * -------------------------------------------------------------------------*/
void W4Integration_Start(void)
{
    W4Motor_Stop();              /* hardware safe state: PWM already zero from MX */
    W4Board_EnableTiming();      /* DWT cycle counter (member6 port) */
    W4Calibration_Apply();       /* geometry/signs; motion locked until W4_CALIBRATED */
    W4Nav_Init();                /* zero nav state, motors stopped */
    W4Clock_Init();              /* member2 64-bit DWT clock */

    /* Pinmux/clocks/DMA/NVIC only. UART RX, LiDAR DMA start inside their tasks. */
    Week3Board_InitBluetooth();
    Week3Board_InitLidar();
    if (Week3Board_InitI2c() != HAL_OK) W4Rtos_Fatal(0x201U, 0U);
    if (week3_init_errors != 0U)        W4Rtos_Fatal(0x202U, week3_init_errors);

    W4InitResult r = W4Rtos_Init();   /* kernel + Q_Lidar/Q_Cmd/Q_Path + sem + mutex + 7 tasks */
    if (r != W4_INIT_OK) W4Rtos_Fatal((uint32_t)r, 0U);

    s_tx_mbox = osMessageQueueNew(TX_MBOX_DEPTH, TX_LINE_MAX, NULL);
    if (s_tx_mbox == NULL) W4Rtos_Fatal(0x203U, 0U);


    /* Sem_ImuTick exists; the 100 Hz ISR saturates it until the IMU task runs. */
    Week3Board_StartImuTimer();
    if (week3_init_errors != 0U) W4Rtos_Fatal(0x204U, week3_init_errors);

    W4Rtos_Start();              /* does not return */
}
