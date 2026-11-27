#include "w4_runtime.h"
#include "w4_i2c_bus.h"
#include "w4_oled.h"
#include "week3_board.h"
#include <assert.h>
#include <setjmp.h>

typedef enum { NORMAL, NACK, START_BUSY, START_ERROR, NO_IRQ, WRONG_IRQ_FIRST } Scenario;
static Scenario scenario;
static I2C_TypeDef i2c_registers;
I2C_HandleTypeDef hi2c1 = {.Instance = &i2c_registers, .State = HAL_I2C_STATE_READY};
static uint32_t ticks, flags, irq_mask, acquires, releases, hal_starts, flags_set, resets;
static uint32_t lock_delay, delays, change_at;
#if W4_OLED_CONTROLLER == W4_OLED_SSD1306
static uint32_t oled_frames, display_chunks;
#endif
static uint16_t last_addr, last_register, gpio_released = GPIO_PIN_8 | GPIO_PIN_9;
static unsigned scl_pulses, sda_release_after;
static bool locked, irq_context, busy, stuck_scl, stuck_sda, pending_irq, abort_fails;
static bool ev_enabled = true, er_enabled = true, lock_fails, oled_test, task_exit;
static void (*completion)(I2C_HandleTypeDef *);
static jmp_buf end_task;
static const osThreadId_t thread = (void *)1;

uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1; }
void __set_PRIMASK(uint32_t mask) { irq_mask = mask; }
uint32_t __get_IPSR(void) { return irq_context; }
uint32_t osKernelGetTickCount(void) { return ticks; }
uint32_t osKernelGetTickFreq(void) { return 1000; }
osKernelState_t osKernelGetState(void) { return osKernelRunning; }
osThreadId_t osThreadGetId(void) { assert(irq_mask == 0); return thread; }

osMutexId_t osMutexNew(const osMutexAttr_t *attr)
{
    assert(attr->attr_bits == osMutexPrioInherit);
    return (void *)2;
}

osStatus_t osMutexAcquire(osMutexId_t mutex, uint32_t timeout)
{
    assert(mutex == Mtx_I2C1 && !locked && !irq_context && timeout > 0);
    if (lock_fails || lock_delay >= timeout) {
        ticks += timeout;
        return osErrorTimeout;
    }
    ticks += lock_delay;
    locked = true;
    ++acquires;
    return osOK;
}

osStatus_t osMutexRelease(osMutexId_t mutex)
{
    assert(mutex == Mtx_I2C1 && locked && !irq_context && hi2c1.pBuffPtr == NULL);
    locked = false;
    ++releases;
    return osOK;
}

uint32_t osThreadFlagsSet(osThreadId_t id, uint32_t bits)
{
    W4I2cStats observed;
    W4I2c_GetStats(&observed);
    assert(id == thread && locked && observed.owner == thread);
    ++flags_set;
    return flags |= bits;
}

uint32_t osThreadFlagsClear(uint32_t bits)
{
    uint32_t old = flags;
    flags &= ~bits;
    return old;
}

uint32_t osThreadFlagsWait(uint32_t bits, uint32_t options, uint32_t timeout)
{
    assert(options == osFlagsWaitAny && locked && timeout != 0);
    if (scenario == NO_IRQ) {
        ticks += timeout;
        pending_irq = true;
        return osFlagsErrorTimeout;
    }
    assert(timeout >= 2 && ev_enabled && er_enabled);
    ticks += 2;
    irq_context = true;
    if (scenario == WRONG_IRQ_FIRST) {
        W4I2c_OnMasterTxComplete(&hi2c1);
        W4I2c_OnAbortComplete(&hi2c1);
        assert(flags == 0);
    }
    hi2c1.State = HAL_I2C_STATE_READY;
    busy = false;
    if (scenario == NACK) {
        hi2c1.ErrorCode = HAL_I2C_ERROR_AF;
        W4I2c_OnError(&hi2c1);
    } else {
        completion(&hi2c1);
    }
    irq_context = false;
    assert((flags & bits) != 0);
    return osThreadFlagsClear(bits);
}

osStatus_t osDelay(uint32_t delay)
{
    assert(locked);
    ticks += delay;
    return osOK;
}

osStatus_t osDelayUntil(uint32_t deadline)
{
    assert(!locked && deadline - ticks < 100);
    ticks = deadline;
    unsigned limit = W4_OLED_HEIGHT == 32 ? 120U : 160U;
    if (++delays == limit)
        longjmp(end_task, 1);
    return osOK;
}

void osThreadExit(void)
{
    task_exit = true;
    longjmp(end_task, 1);
}

void mock_reset(void)
{
    assert(locked && !ev_enabled && !er_enabled);
    ++resets;
    busy = false;
}

FlagStatus mock_bus_flag(void) { return busy ? SET : RESET; }
uint32_t HAL_I2C_GetState(I2C_HandleTypeDef *h) { return h->State; }
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *h) { return h->ErrorCode; }
void HAL_NVIC_DisableIRQ(IRQn_Type irq) { *(irq == I2C1_EV_IRQn ? &ev_enabled : &er_enabled) = false; }
void HAL_NVIC_EnableIRQ(IRQn_Type irq) { *(irq == I2C1_EV_IRQn ? &ev_enabled : &er_enabled) = true; }
void HAL_NVIC_ClearPendingIRQ(IRQn_Type irq) { (void)irq; pending_irq = false; }

HAL_StatusTypeDef HAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *h, uint16_t addr)
{
    assert(locked && addr == last_addr && !ev_enabled && !er_enabled && pending_irq);
    if (abort_fails)
        return HAL_ERROR;
    W4I2c_OnAbortComplete(h);
    return HAL_OK;
}

static HAL_StatusTypeDef start(I2C_HandleTypeDef *h, uint16_t addr, uint8_t *buf,
                               void (*callback)(I2C_HandleTypeDef *))
{
    assert(locked && h == &hi2c1 && !pending_irq && !busy && flags == 0);
    ++hal_starts;
    last_addr = addr;
    if (scenario == START_BUSY)
        return HAL_BUSY;
    if (scenario == START_ERROR)
        return HAL_ERROR;
    h->pBuffPtr = buf;
    h->ErrorCode = 0;
    h->State = HAL_I2C_STATE_BUSY;
    busy = true;
    completion = callback;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *h, uint16_t addr,
                                            uint8_t *buf, uint16_t len)
{
#if W4_OLED_CONTROLLER == W4_OLED_SSD1306
    if (oled_test) {
        assert(addr == 0x78 && len <= 23);
        ++oled_frames;
        if (len == 23) {
            assert(buf[0] == 0x80 && buf[2] == 0x80 && buf[4] == 0x80 && buf[6] == 0x40);
            assert(buf[1] >= 0xb0 && buf[1] < 0xb0 + W4_OLED_HEIGHT / 8U);
            assert(buf[3] == 0 && buf[5] >= 0x10 && buf[5] < 0x18);
            if (display_chunks < W4_OLED_HEIGHT) {
                for (unsigned i = 7; i < len; ++i)
                    assert(buf[i] == 0);
            }
            ++display_chunks;
        }
    }
#else
    (void)len;
#endif
    return start(h, addr, buf, W4I2c_OnMasterTxComplete);
}

HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *h, uint16_t addr, uint16_t reg,
                                     uint16_t size, uint8_t *buf, uint16_t len)
{
    assert(size == I2C_MEMADD_SIZE_8BIT && len > 0);
    last_register = reg;
    HAL_StatusTypeDef started = start(h, addr, buf, W4I2c_OnMemRxComplete);
    if (started == HAL_OK)
        buf[0] = 0x70;
    return started;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *h, uint16_t addr, uint16_t reg,
                                      uint16_t size, uint8_t *buf, uint16_t len)
{
    assert(size == I2C_MEMADD_SIZE_8BIT && len > 0);
    last_register = reg;
    return start(h, addr, buf, W4I2c_OnMemTxComplete);
}

void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState value)
{
    assert(port == GPIOB && locked);
    if (value == GPIO_PIN_SET) {
        if (pin == GPIO_PIN_8 && (gpio_released & pin) == 0U && ++scl_pulses == sda_release_after)
            stuck_sda = false;
        gpio_released |= pin;
    } else {
        gpio_released &= (uint16_t)~pin;
    }
}

GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin)
{
    assert(port == GPIOB);
    bool held = pin == GPIO_PIN_8 ? stuck_scl : stuck_sda;
    return !held && (gpio_released & pin) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *gpio)
{
    assert(port == GPIOB && locked && !ev_enabled && !er_enabled);
    assert(gpio->Pin == (GPIO_PIN_8 | GPIO_PIN_9) && gpio->Mode == GPIO_MODE_OUTPUT_OD);
}

HAL_StatusTypeDef Week3Board_InitI2c(void)
{
    assert(locked && !ev_enabled && !er_enabled && !pending_irq);
    hi2c1.State = HAL_I2C_STATE_READY;
    ev_enabled = er_enabled = true;
    return HAL_OK;
}

bool W4Oled_ReadNavSnapshot(W4OledNavSnapshot *out)
{
    assert(oled_test && !locked);
    *out = (W4OledNavSnapshot){
        .x_mm = delays >= change_at ? 700 : 0, .y_mm = 1400, .theta_mrad = 1570,
        .target_x_mm = -700, .target_y_mm = 700, .state = W4_OLED_NAV_TRACKING,
        .fault = 0, .pose_valid = true, .target_valid = true
    };
    return true;
}

static void test_bus(void)
{
    uint8_t buf[14] = {0};
    W4I2cStats stats;
    ticks = UINT32_MAX - 1U;
    irq_context = true;
    assert(!W4I2c_MemRead(0x68, 0x75, buf, 1, 8));
    irq_context = false;
    assert(W4I2c_MemRead(0x68, 0x75, buf, 1, 8));
    assert(ticks == 0 && buf[0] == 0x70 && last_addr == 0xd0 && last_register == 0x75);
    assert(W4I2c_MemWrite(0x69, 0x6b, buf, 1, 8));
    assert(last_addr == 0xd2 && last_register == 0x6b);
    assert(W4I2c_Write(0x3d, buf, sizeof buf, 8) && last_addr == 0x7a);
    lock_fails = true;
    assert(!W4I2c_MemRead(0x68, 0x3b, buf, sizeof buf, 4));
    assert(hal_starts == 3 && acquires == releases);
    lock_fails = false;
    lock_delay = 2;
    scenario = START_BUSY;
    assert(!W4I2c_MemRead(0x68, 0x3b, buf, sizeof buf, 6));
    scenario = START_ERROR;
    assert(!W4I2c_MemRead(0x68, 0x3b, buf, sizeof buf, 6));
    scenario = NACK;
    assert(!W4I2c_MemRead(0x68, 0x3b, buf, sizeof buf, 6));
    W4I2c_GetStats(&stats);
    assert(stats.last_hal_error == HAL_I2C_ERROR_AF && stats.available);
    scenario = WRONG_IRQ_FIRST;
    assert(W4I2c_MemRead(0x68, 0x3b, buf, sizeof buf, 6));
    assert(flags == 0 && acquires == releases);
    scenario = NO_IRQ;
    uint32_t start_tick = ticks;
    assert(!W4I2c_MemRead(0x68, 0x3b, buf, sizeof buf, 7));
    assert(ticks - start_tick == 7 && !ev_enabled && !er_enabled && !pending_irq);
    unsigned notified = flags_set;
    W4I2c_OnMemRxComplete(&hi2c1);
    W4I2c_OnAbortComplete(&hi2c1);
    assert(flags_set == notified && flags == 0);
    assert(!W4I2c_MemRead(0x68, 0x3b, buf, sizeof buf, 7)); /* offline */
    lock_delay = 0;
    stuck_scl = true;
    assert(!W4I2c_Recover(60) && gpio_released == (GPIO_PIN_8 | GPIO_PIN_9));
    stuck_scl = false;
    stuck_sda = true;
    assert(!W4I2c_Recover(6));
    scl_pulses = 0;
    sda_release_after = 3;
    assert(W4I2c_Recover(60) && scl_pulses == 4);
    scenario = NORMAL;
    assert(W4I2c_MemRead(0x68, 0x3b, buf, sizeof buf, 6));
    scenario = NO_IRQ;
    abort_fails = true;
    assert(!W4I2c_MemRead(0x68, 0x3b, buf, sizeof buf, 6));
    assert(!ev_enabled && !er_enabled && !pending_irq);
    assert(W4I2c_Recover(60));
    scenario = NORMAL;
    busy = true;
    unsigned started = hal_starts;
    assert(!W4I2c_MemRead(0x68, 0x3b, buf, sizeof buf, 6));
    assert(hal_starts == started && W4I2c_Recover(60));
    assert(W4I2c_MemRead(0x68, 0x3b, buf, sizeof buf, 6));
    W4I2c_GetStats(&stats);
    assert(stats.completed == 6 && stats.imu.lock_timeouts == 1 && stats.imu.max_wait_ticks == 4);
    assert(stats.imu.max_hold_ticks == 6 && stats.oled.acquisitions == 1);
    assert(stats.hal_busy == 2 && stats.start_errors == 1 && stats.nacks == 1);
    assert(stats.transfer_timeouts == 2 && stats.abort_callbacks == 1 && stats.ignored_callbacks == 4);
    assert(stats.recoveries == 5 && stats.recovery_failures == 2 && stats.peripheral_resets == resets);
    assert(stats.owner == NULL && stats.available && acquires == releases);
}

static void test_oled(void)
{
    oled_test = true;
    change_at = W4_OLED_HEIGHT + 23U;
    ticks = UINT32_MAX - 100U;
    unsigned before = hal_starts;
    if (setjmp(end_task) == 0)
        W4OledTask(NULL);
    W4OledStats report;
    W4Oled_GetStats(&report);
#if W4_OLED_CONTROLLER == W4_OLED_SSD1306
    assert(!task_exit && report.status == W4_OLED_READY && report.periods == delays);
    assert(report.io_failures == 0 && report.missed_periods == 0);
    assert(report.chunks_sent == display_chunks && display_chunks > W4_OLED_HEIGHT);
    assert(oled_frames == display_chunks + 3U && oled_frames == hal_starts - before);
    assert(oled_frames == delays && acquires == releases); /* sustained shared bus traffic */
#else
    assert(task_exit && report.status == W4_OLED_UNCONFIGURED && hal_starts == before);
#endif
}

int main(void)
{
    W4I2cSync_Init();
    assert(Mtx_I2C1 != NULL);
    test_bus();
    test_oled();
    return 0;
}

void W4Runtime_Begin(W4TaskId id,uint32_t at){(void)id;(void)at;}
void W4Runtime_End(W4TaskId id,uint32_t d){(void)id;(void)d;}

osStatus_t osThreadSuspend(osThreadId_t id){assert(id==thread&&!locked);task_exit=true;longjmp(end_task,1);return osOK;}
