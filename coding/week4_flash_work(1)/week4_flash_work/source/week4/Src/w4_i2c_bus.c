#include "w4_i2c_bus.h"
#include "week3_board.h"

#define I2C_DONE_FLAG (1UL << 16)
#define BUS_PINS (GPIO_PIN_8 | GPIO_PIN_9)

typedef enum { NO_TRANSFER, MEM_RX, MEM_TX, MASTER_TX, ABORTING } Operation;
typedef enum { PENDING, COMPLETE, TRANSFER_ERROR, ABORTED } Result;

osMutexId_t Mtx_I2C1;
static W4I2cStats stats;
static volatile Operation active;
static volatile Result result;
static osThreadId_t volatile waiter;

static uint32_t irq_save(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    return mask;
}

static void irq_restore(uint32_t mask)
{
    __set_PRIMASK(mask);
}

static uint32_t remaining(uint32_t deadline)
{
    uint32_t ticks = deadline - osKernelGetTickCount();
    return ticks <= INT32_MAX ? ticks : 0;
}

static bool line_high(uint16_t pin)
{
    return HAL_GPIO_ReadPin(GPIOB, pin) == GPIO_PIN_SET;
}

static bool bus_idle(void)
{
    return HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY &&
           __HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) == RESET &&
           line_high(GPIO_PIN_8) && line_high(GPIO_PIN_9);
}

static void mask_i2c_irqs(void)
{
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
    __DSB();
    __ISB();
}

static void clear_i2c_pending(void)
{
    HAL_NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
    HAL_NVIC_ClearPendingIRQ(I2C1_ER_IRQn);
}

/* Called under the mutex. I2C1 has no DMA clients. Reset stops requests,
 * cancels access to the caller's buffer and drops the old IRQ epoch. */
static void stop_peripheral(void)
{
    mask_i2c_irqs();
    __HAL_I2C_DISABLE_IT(&hi2c1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);
    __HAL_I2C_DISABLE(&hi2c1);
    __HAL_RCC_I2C1_FORCE_RESET();
    __DSB();
    __HAL_RCC_I2C1_RELEASE_RESET();
    clear_i2c_pending();
    hi2c1.pBuffPtr = NULL;
    hi2c1.Mode = HAL_I2C_MODE_NONE;
    hi2c1.State = HAL_I2C_STATE_RESET;
    hi2c1.Lock = HAL_UNLOCKED;
    uint32_t mask = irq_save();
    stats.available = false;
    ++stats.peripheral_resets;
    irq_restore(mask);
}

void W4I2cSync_Init(void)
{
    if (Mtx_I2C1 != NULL)
        return;
    static const osMutexAttr_t attributes = {
        .name = "Mtx_I2C1", .attr_bits = osMutexPrioInherit
    };
    Mtx_I2C1 = osMutexNew(&attributes);
    stats.available = Mtx_I2C1 != NULL &&
                      HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY;
}

static void record_wait(W4I2cTiming *timing, uint32_t ticks, bool acquired)
{
    if (acquired)
        ++timing->acquisitions;
    else
        ++timing->lock_timeouts;
    timing->total_wait_ticks += ticks;
    if (ticks > timing->max_wait_ticks)
        timing->max_wait_ticks = ticks;
}

static bool acquire(uint32_t timeout_ms, W4I2cTiming *client,
                    uint32_t *deadline, uint32_t *held_since)
{
    if (__get_IPSR() != 0U || Mtx_I2C1 == NULL || timeout_ms == 0 ||
        osKernelGetState() != osKernelRunning)
        return false;

    uint64_t ticks = ((uint64_t)timeout_ms * osKernelGetTickFreq() + 999U) / 1000U;
    if (ticks == 0 || ticks > INT32_MAX)
        return false;

    uint32_t start = osKernelGetTickCount();
    *deadline = start + (uint32_t)ticks;
    uint32_t budget = remaining(*deadline);
    bool acquired = budget != 0 && osMutexAcquire(Mtx_I2C1, budget) == osOK;
    *held_since = osKernelGetTickCount();
    osThreadId_t owner = acquired ? osThreadGetId() : NULL;
    uint32_t mask = irq_save();
    uint32_t waited = *held_since - start;
    record_wait(&stats.all, waited, acquired);
    if (client != NULL)
        record_wait(client, waited, acquired);
    if (acquired) {
        stats.last_wait_ticks = waited;
        stats.owner = owner;
    }
    irq_restore(mask);
    return acquired;
}

static void release(W4I2cTiming *client, uint32_t held_since)
{
    uint32_t held = osKernelGetTickCount() - held_since;
    uint32_t mask = irq_save();
    stats.last_hold_ticks = held;
    if (held > stats.all.max_hold_ticks)
        stats.all.max_hold_ticks = held;
    if (client != NULL && held > client->max_hold_ticks)
        client->max_hold_ticks = held;
    stats.owner = NULL;
    irq_restore(mask);
    (void)osMutexRelease(Mtx_I2C1);
}

static void notify(I2C_HandleTypeDef *h, Operation expected, Result event)
{
    if (h != &hi2c1)
        return;
    if (waiter == NULL || result != PENDING ||
        (expected != NO_TRANSFER && active != expected) ||
        (expected == NO_TRANSFER && (active == NO_TRANSFER || active == ABORTING))) {
        ++stats.ignored_callbacks;
        return;
    }
    result = event;
    if (event == ABORTED)
        ++stats.abort_callbacks;
    (void)osThreadFlagsSet(waiter, I2C_DONE_FLAG);
}

void W4I2c_OnMemRxComplete(I2C_HandleTypeDef *h) { notify(h, MEM_RX, COMPLETE); }
void W4I2c_OnMemTxComplete(I2C_HandleTypeDef *h) { notify(h, MEM_TX, COMPLETE); }
void W4I2c_OnMasterTxComplete(I2C_HandleTypeDef *h) { notify(h, MASTER_TX, COMPLETE); }
void W4I2c_OnError(I2C_HandleTypeDef *h) { notify(h, NO_TRANSFER, TRANSFER_ERROR); }
void W4I2c_OnAbortComplete(I2C_HandleTypeDef *h) { notify(h, ABORTING, ABORTED); }

static HAL_StatusTypeDef start_transfer(Operation operation, uint16_t addr,
                                        uint8_t reg, uint8_t *buf, uint16_t len)
{
    switch (operation) {
    case MEM_RX:
        return HAL_I2C_Mem_Read_IT(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len);
    case MEM_TX:
        return HAL_I2C_Mem_Write_IT(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len);
    default:
        return HAL_I2C_Master_Transmit_IT(&hi2c1, addr, buf, len);
    }
}

static void abort_transfer(uint16_t addr)
{
    mask_i2c_irqs();
    if (result != PENDING && bus_idle())
        return;
    active = ABORTING;
    result = PENDING;
    /* The Week 3 IT HAL calls AbortCplt synchronously and disables PE.
     * Reset I2C1 even if HAL cannot abort, so old IRQs cannot survive. */
    (void)HAL_I2C_Master_Abort_IT(&hi2c1, addr);
    stop_peripheral();
}

static void retire_transfer(void)
{
    mask_i2c_irqs();
    __HAL_I2C_DISABLE_IT(&hi2c1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);
    clear_i2c_pending();
    hi2c1.pBuffPtr = NULL;
    active = NO_TRANSFER;
    waiter = NULL;
    (void)osThreadFlagsClear(I2C_DONE_FLAG);
    if (stats.available) {
        HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
        HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    }
}

static bool wait_for_idle(uint32_t deadline)
{
    /* HAL can report completion just after requesting STOP, before BUSY drops. */
    while (!bus_idle()) {
        if (remaining(deadline) == 0 || osDelay(1U) != osOK)
            return false;
    }
    return true;
}

static void record_transfer_result(Result terminal, bool ok, bool timed_out)
{
    uint32_t hal_error = HAL_I2C_GetError(&hi2c1);
    uint32_t mask = irq_save();
    if (terminal == TRANSFER_ERROR) {
        ++stats.transfer_errors;
        if ((hal_error & HAL_I2C_ERROR_AF) != 0U)
            ++stats.nacks;
    }
    if (ok)
        ++stats.completed;
    else
        stats.last_hal_error = hal_error;
    if (timed_out)
        ++stats.transfer_timeouts;
    irq_restore(mask);
}

static bool transfer(Operation operation, uint8_t addr7, uint8_t reg,
                     uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    if (addr7 > 0x7fU || buf == NULL || len == 0)
        return false;
    W4I2cTiming *client = (addr7 == 0x68U || addr7 == 0x69U) ? &stats.imu : NULL;
    if (addr7 == 0x3cU || addr7 == 0x3dU)
        client = &stats.oled;
    uint32_t deadline, held_since;
    if (!acquire(timeout_ms, client, &deadline, &held_since))
        return false;

    bool ok = false;
    if (stats.available && remaining(deadline) != 0U) {
        uint16_t addr = (uint16_t)addr7 << 1U;
        (void)osThreadFlagsClear(I2C_DONE_FLAG);
        waiter = osThreadGetId();
        result = PENDING;
        active = operation;
        /* The HAL IT start functions have their own long BUSY spin.
         * The sole master checks idle before calling them. */
        HAL_StatusTypeDef started = bus_idle() ? start_transfer(operation, addr, reg, buf, len)
                                               : HAL_BUSY;
        if (started == HAL_OK) {
            uint32_t budget = remaining(deadline);
            if (budget != 0U && result == PENDING)
                (void)osThreadFlagsWait(I2C_DONE_FLAG, osFlagsWaitAny, budget);

            Result terminal = result;
            bool stopped = terminal != PENDING && wait_for_idle(deadline);
            bool timed_out = !stopped || remaining(deadline) == 0U;
            ok = terminal == COMPLETE && !timed_out;
            record_transfer_result(terminal, ok, timed_out);
            if (!stopped)
                abort_transfer(addr);
        } else {
            uint32_t mask = irq_save();
            stats.last_hal_error = HAL_I2C_GetError(&hi2c1);
            if (started == HAL_BUSY)
                ++stats.hal_busy;
            else
                ++stats.start_errors;
            irq_restore(mask);
            if (!bus_idle())
                stop_peripheral();
        }
        retire_transfer();
    }
    release(client, held_since);
    return ok;
}

bool W4I2c_MemRead(uint8_t addr7, uint8_t reg, uint8_t *buf, uint16_t len,
                   uint32_t timeout_ms)
{
    return transfer(MEM_RX, addr7, reg, buf, len, timeout_ms);
}

bool W4I2c_MemWrite(uint8_t addr7, uint8_t reg, const uint8_t *buf, uint16_t len,
                    uint32_t timeout_ms)
{
    return transfer(MEM_TX, addr7, reg, (uint8_t *)buf, len, timeout_ms);
}

bool W4I2c_Write(uint8_t addr7, const uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    return transfer(MASTER_TX, addr7, 0, (uint8_t *)buf, len, timeout_ms);
}

static bool pause_edge(uint32_t deadline)
{
    return remaining(deadline) > 2U && osDelay(2U) == osOK &&
           remaining(deadline) != 0U;
}

/* Week 3 bus-clear sequence on the original pins, with bounded task delays. */
static bool clear_lines(uint32_t deadline)
{
    GPIO_InitTypeDef gpio = {
        .Pin = BUS_PINS, .Mode = GPIO_MODE_OUTPUT_OD, .Pull = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_HIGH
    };
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOB, BUS_PINS, GPIO_PIN_SET);
    HAL_GPIO_Init(GPIOB, &gpio);
    if (!pause_edge(deadline) || !line_high(GPIO_PIN_8))
        return false;
    for (unsigned pulse = 0; pulse < 9U && !line_high(GPIO_PIN_9); ++pulse) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        if (!pause_edge(deadline))
            return false;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        if (!pause_edge(deadline) || !line_high(GPIO_PIN_8))
            return false;
    }
    if (!line_high(GPIO_PIN_9))
        return false;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    if (!pause_edge(deadline))
        return false;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    if (!pause_edge(deadline))
        return false;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    if (!pause_edge(deadline) || !line_high(GPIO_PIN_8))
        return false;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    return pause_edge(deadline) && line_high(GPIO_PIN_9);
}

bool W4I2c_Recover(uint32_t timeout_ms)
{
    uint32_t deadline, held_since;
    if (!acquire(timeout_ms, &stats.recovery, &deadline, &held_since))
        return false;
    bool recovered = false;
    if (remaining(deadline) != 0U) {
        ++stats.recoveries;
        stop_peripheral();
        bool released = clear_lines(deadline);
        HAL_GPIO_WritePin(GPIOB, BUS_PINS, GPIO_PIN_SET);
        if (released && Week3Board_InitI2c() == HAL_OK && bus_idle() &&
            remaining(deadline) != 0U) {
            stats.available = true;
            recovered = true;
        } else {
            stop_peripheral();
            ++stats.recovery_failures;
        }
    }
    release(&stats.recovery, held_since);
    return recovered;
}

void W4I2c_GetStats(W4I2cStats *out)
{
    if (out != NULL) {
        uint32_t mask = irq_save();
        *out = stats;
        irq_restore(mask);
    }
}
