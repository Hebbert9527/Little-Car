#include "lidar_dma.h"
#include "lidar_parser.h"
#include "w4_queues.h"
#include "w4_runtime.h"

#include <string.h>

/* Integration: member2 creates threads with argument=NULL, so fall back to the
   board LiDAR UART (USART1) when no handle is passed. Wiring is unchanged. */
extern UART_HandleTypeDef huart1;

enum
{
    HAL_OPERATION_MS = 50,
    STOP_SETTLE_MS = 15,
    RESET_SETTLE_MS = 510,
    DESCRIPTOR_TIMEOUT_MS = 10000,
    HALF_TIMEOUT_MS = 250,
    RETRY_DELAY_MS = 100,
    RETRY_COOLDOWN_MS = 2000,
    FAST_ATTEMPTS = 3
};

static uint8_t stop_command[] = {0xA5, 0x25};
static uint8_t reset_command[] = {0xA5, 0x40};
static uint8_t scan_command[] = {0xA5, 0x20};
static const uint8_t scan_descriptor[] = {0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};

static UART_HandleTypeDef *uart;
static uint8_t rx_ring[LIDAR_DMA_SIZE];
static uint8_t staging[LIDAR_DMA_HALF_SIZE]; /* Owned exclusively by the LiDAR task. */
static volatile LidarDma_Stats stats;
static uint32_t state_started_ms;
static uint32_t attempt;
static volatile bool receiving;
static volatile bool uart_fault;
static volatile bool sequence_fault;
static volatile uint32_t produced;
static volatile uint32_t last_event_ms;
static volatile bool had_event;
static uint32_t consumed;
static size_t prefix_bytes;

static void enter(LidarDma_State state)
{
    state_started_ms = HAL_GetTick();
    stats.state = state;
}

static bool elapsed(uint32_t since, uint32_t duration)
{
    return HAL_GetTick() - since >= duration;
}

static bool rx_running(void)
{
    return (uart->Instance->CR3 & USART_CR3_DMAR) != 0U &&
           (uart->hdmarx->Instance->CR & DMA_SxCR_EN) != 0U;
}

static bool dma_quiet(void)
{
    return (uart->hdmarx->Instance->CR & DMA_SxCR_EN) == 0U &&
           uart->hdmarx->State == HAL_DMA_STATE_READY;
}

static bool tc_pending(void)
{
    DMA_HandleTypeDef *dma = uart->hdmarx;
    return __HAL_DMA_GET_FLAG(dma, __HAL_DMA_GET_TC_FLAG_INDEX(dma)) != 0U;
}

static bool ht_pending(void)
{
    DMA_HandleTypeDef *dma = uart->hdmarx;
    return __HAL_DMA_GET_FLAG(dma, __HAL_DMA_GET_HT_FLAG_INDEX(dma)) != 0U;
}

static void record_uart_error(uint32_t code)
{
    ++stats.uart_errors;
    stats.last_uart_error = code;
    uart_fault = true;
}

static void recover(void)
{
    if (stats.state == LIDAR_DMA_STREAM)
        ++stats.stream_breaks;

    /* Keep the UART/DMA IRQs out while HAL changes the abort callbacks. */
    const uint32_t irq_mask = __get_PRIMASK();
    __disable_irq();
    /* HAL can stop DMAR before its asynchronous error callback runs. */
    if (receiving && !uart_fault && uart->ErrorCode != HAL_UART_ERROR_NONE)
        record_uart_error(uart->ErrorCode);
    receiving = false;
    const HAL_StatusTypeDef result = HAL_UART_Abort_IT(uart);
    __set_PRIMASK(irq_mask);

    /* No producer can enqueue after receiving was cleared. Invalidate queued
       descriptors and partial records before another DMA session starts. */
    LidarParser_ResetStream();
    prefix_bytes = 0U;
    if (W4Queue_LidarReset() != osOK)
    {
        ++stats.queue_errors;
        enter(LIDAR_DMA_FAILED);
        return;
    }

    if (result != HAL_OK)
    {
        ++stats.hal_failures;
        enter(LIDAR_DMA_FAILED);
    }
    else
    {
        enter(LIDAR_DMA_WAIT_ABORT);
    }
}

static void overrun(void)
{
    ++stats.overruns;
    recover();
}

static bool check_receiver(void)
{
    if (uart_fault)
    {
        recover();
        return false;
    }
    if (!rx_running())
    {
        ++stats.rx_stops;
        recover();
        return false;
    }
    if (sequence_fault)
    {
        overrun();
        return false;
    }
    return true;
}

/* Retry HAL_BUSY within the current state's time budget. */
static bool operation_ok(HAL_StatusTypeDef result)
{
    if (result == HAL_OK)
        return true;
    if (result == HAL_BUSY)
    {
        ++stats.hal_busy;
        if (!elapsed(state_started_ms, HAL_OPERATION_MS))
            return false;
    }
    ++stats.hal_failures;
    recover();
    return false;
}

static void record_half(bool first_half)
{
    const uint32_t sequence = produced;
    if (first_half != ((sequence & 1U) == 0U) || (first_half ? tc_pending() : ht_pending()))
    {
        sequence_fault = true;
        return;
    }

    const uint32_t now = HAL_GetTick();
    if (sequence != 0U)
    {
        const uint32_t interval = now - last_event_ms;
        stats.last_half_interval_ms = interval;
        if (interval != 0U &&
            (stats.min_half_interval_ms == 0U || interval < stats.min_half_interval_ms))
            stats.min_half_interval_ms = interval;
    }
    last_event_ms = now;
    had_event = true;
    produced = sequence + 1U;

    /* The queue contains an owned descriptor, never a pointer into DMA RAM.
       The sequence advances even when the queue is full. */
    const W4LidarBlock block = {
        .seq = produced, .stamp_ms = now,
        .offset = first_half ? 0U : LIDAR_DMA_HALF_SIZE,
        .length = LIDAR_DMA_HALF_SIZE, .half = first_half ? 0U : 1U
    };
    (void)W4Queue_LidarPut(&block); /* ISR: CMSIS timeout is always zero. */
}

void LidarDma_OnHalf(UART_HandleTypeDef *huart)
{
    if (receiving && huart == uart)
    {
        ++stats.ht_count;
        record_half(true);
    }
}

void LidarDma_OnFull(UART_HandleTypeDef *huart)
{
    if (receiving && huart == uart)
    {
        ++stats.tc_count;
        record_half(false);
    }
}

void LidarDma_OnError(UART_HandleTypeDef *huart)
{
    if (receiving && huart == uart)
        record_uart_error(huart->ErrorCode);
}

void LidarDma_Init(UART_HandleTypeDef *huart)
{
    receiving = false;
    uart = huart;
    stats = (LidarDma_Stats){0};
    produced = consumed = last_event_ms = 0U;
    had_event = false;
    prefix_bytes = 0U;
    uart_fault = sequence_fault = false;
}

bool LidarDma_Start(void)
{
    if (!W4Queues_Ready() || stats.state != LIDAR_DMA_IDLE || uart == NULL || uart->Instance == NULL ||
        uart->hdmarx == NULL || uart->hdmarx->Instance == NULL)
        return false;

    const DMA_HandleTypeDef *dma = uart->hdmarx;
    if (uart->Init.BaudRate != 460800U || uart->Init.WordLength != UART_WORDLENGTH_8B ||
        uart->Init.StopBits != UART_STOPBITS_1 || uart->Init.Parity != UART_PARITY_NONE ||
        uart->Init.Mode != UART_MODE_TX_RX || uart->Init.HwFlowCtl != UART_HWCONTROL_NONE ||
        dma->Init.Mode != DMA_CIRCULAR || dma->Init.Direction != DMA_PERIPH_TO_MEMORY ||
        dma->Init.PeriphInc != DMA_PINC_DISABLE || dma->Init.MemInc != DMA_MINC_ENABLE ||
        dma->Init.PeriphDataAlignment != DMA_PDATAALIGN_BYTE ||
        dma->Init.MemDataAlignment != DMA_MDATAALIGN_BYTE ||
        dma->Init.FIFOMode != DMA_FIFOMODE_DISABLE)
        return false;

    if (W4Queue_LidarReset() != osOK) return false;
    attempt = 1U;
    enter(LIDAR_DMA_SEND_SYNC);
    return true;
}

static void send_scan(void)
{
    if (!receiving)
    {
        if (!dma_quiet())
        {
            (void)operation_ok(HAL_BUSY);
            return;
        }
        uart_fault = sequence_fault = false;
        produced = consumed = 0U;
        had_event = false;
        receiving = true;
        /* A scan point can follow the final descriptor byte immediately. */
        const HAL_StatusTypeDef result = HAL_UART_Receive_DMA(uart, rx_ring, LIDAR_DMA_SIZE);
        if (result != HAL_OK)
        {
            receiving = false;
            (void)operation_ok(result);
            return;
        }
    }

    if (check_receiver() &&
        operation_ok(HAL_UART_Transmit_IT(uart, scan_command, sizeof scan_command)))
        enter(LIDAR_DMA_WAIT_DESCRIPTOR);
}

static void poll_descriptor(void)
{
    if (!check_receiver())
        return;
    if (elapsed(state_started_ms, DESCRIPTOR_TIMEOUT_MS))
    {
        ++stats.descriptor_timeouts;
        recover();
        return;
    }
    if (uart->gState != HAL_UART_STATE_READY)
        return;

    const uint32_t remaining = __HAL_DMA_GET_COUNTER(uart->hdmarx);
    if (produced > 1U || remaining == 0U || tc_pending())
    {
        overrun();
        return;
    }
    /* NDTR follows peripheral reads. In direct mode one byte can still
       be in flight to SRAM, so wait for the first scan byte as well. */
    if (remaining >= LIDAR_DMA_SIZE - sizeof scan_descriptor)
        return;

    __DMB();
    const bool valid = memcmp(rx_ring, scan_descriptor, sizeof scan_descriptor) == 0;
    __DMB();
    if (!check_receiver())
        return;
    if (produced > 1U || tc_pending())
    {
        overrun();
    }
    else if (!valid)
    {
        ++stats.bad_descriptors;
        recover();
    }
    else
    {
        prefix_bytes = sizeof scan_descriptor;
        enter(LIDAR_DMA_STREAM);
    }
}

static bool half_stable(uint32_t sequence, bool first_half)
{
    const uint32_t remaining = __HAL_DMA_GET_COUNTER(uart->hdmarx);
    /* Require one byte of progress into the other half: in direct mode the
       last peripheral read can precede its final SRAM write. */
    const bool other_half_active = first_half
        ? remaining > 0U && remaining < LIDAR_DMA_HALF_SIZE
        : remaining > LIDAR_DMA_HALF_SIZE && remaining < LIDAR_DMA_SIZE;
    return other_half_active && !ht_pending() && !tc_pending() && produced == sequence;
}

static void poll_stream(void)
{
    if (!check_receiver())
        return;
    const uint32_t sequence = produced;
    const uint32_t progress_ms = had_event ? last_event_ms : state_started_ms;
    if (elapsed(progress_ms, HALF_TIMEOUT_MS) && sequence == produced)
    {
        ++stats.stream_timeouts;
        recover();
    }
}

void LidarDma_ProcessBlock(const W4LidarBlock *block)
{
    if (stats.state != LIDAR_DMA_STREAM || block == NULL || !check_receiver()) return;
    const uint32_t sequence = block->seq;
    const bool first_half = block->half == 0U;
    if (block->half > 1U || block->length != LIDAR_DMA_HALF_SIZE ||
        block->offset != (first_half ? 0U : LIDAR_DMA_HALF_SIZE) ||
        first_half != ((sequence & 1U) != 0U))
    {
        ++stats.dropped_blocks;
        LidarParser_ResetStream();
        prefix_bytes = 0U;
        overrun(); /* malformed descriptors should never arise in a valid session */
        return;
    }

    if (sequence != consumed + 1U)
    {
        ++stats.sequence_gaps;
        LidarParser_ResetStream();
        prefix_bytes = 0U;
    }
    consumed = sequence;
    const uint32_t age_ms = HAL_GetTick() - block->stamp_ms;
    if (age_ms > stats.max_queue_age_ms) stats.max_queue_age_ms = age_ms;
    const uint32_t remaining = __HAL_DMA_GET_COUNTER(uart->hdmarx);
    if ((first_half && remaining == LIDAR_DMA_HALF_SIZE) ||
        (!first_half && (remaining == LIDAR_DMA_SIZE || remaining == 0U)))
    {
        /* Yield once at the DMA boundary, then recheck. Never spin in a task. */
        (void)osDelay(1U);
        if (!check_receiver()) return;
    }
    if (!half_stable(sequence, first_half))
    {
        ++stats.overruns;
        ++stats.dropped_blocks;
        LidarParser_ResetStream();
        prefix_bytes = 0U;
        return;
    }

    const uint32_t started_ms = HAL_GetTick();
    __DMB();
    memcpy(staging, &rx_ring[block->offset], sizeof staging);
    __DMB();
    const uint32_t copy_ms = HAL_GetTick() - started_ms;
    if (copy_ms > stats.max_copy_ms) stats.max_copy_ms = copy_ms;
    if (!check_receiver())
        return;
    if (!half_stable(sequence, first_half))
    {
        ++stats.overruns;
        ++stats.dropped_blocks;
        LidarParser_ResetStream();
        prefix_bytes = 0U;
        return;
    }

    /* The seven-byte descriptor exists only in session's first half. */
    if (sequence != 1U) prefix_bytes = 0U;
    const size_t len = sizeof staging - prefix_bytes;
    LidarParser_Feed(staging + prefix_bytes, len);
    prefix_bytes = 0U;
    attempt = 1U; /* Verified progress restores fast retries. */
    ++stats.halves_delivered;
    stats.bytes_delivered += (uint32_t)len;
    const uint32_t batch_ms = HAL_GetTick() - started_ms;
    if (batch_ms > stats.max_batch_ms)
        stats.max_batch_ms = batch_ms;
}

void LidarDma_Poll(void)
{
    /* Slow retries use STOP so a late boot response cannot trigger a reset loop. */
    const bool reset_before_scan = attempt > 1U && attempt <= FAST_ATTEMPTS;
    switch (stats.state)
    {
    case LIDAR_DMA_SEND_SYNC:
        if (operation_ok(HAL_UART_Transmit_IT(uart, reset_before_scan ? reset_command : stop_command,
                                              sizeof stop_command)))
            enter(LIDAR_DMA_WAIT_SYNC_TX);
        break;
    case LIDAR_DMA_WAIT_SYNC_TX:
        if (uart->gState == HAL_UART_STATE_READY)
        {
            enter(LIDAR_DMA_WAIT_QUIET);
        }
        else if (elapsed(state_started_ms, HAL_OPERATION_MS))
        {
            ++stats.hal_failures;
            recover();
        }
        break;
    case LIDAR_DMA_WAIT_QUIET:
        if (elapsed(state_started_ms, reset_before_scan ? RESET_SETTLE_MS : STOP_SETTLE_MS))
            enter(LIDAR_DMA_SEND_SCAN);
        break;
    case LIDAR_DMA_SEND_SCAN:
        send_scan();
        break;
    case LIDAR_DMA_WAIT_DESCRIPTOR:
        poll_descriptor();
        break;
    case LIDAR_DMA_STREAM:
        poll_stream();
        break;
    case LIDAR_DMA_WAIT_ABORT:
        if (dma_quiet() && uart->gState == HAL_UART_STATE_READY &&
            uart->RxState == HAL_UART_STATE_READY)
        {
            enter(LIDAR_DMA_RETRY_DELAY);
        }
        else if (elapsed(state_started_ms, HAL_OPERATION_MS))
        {
            ++stats.abort_timeouts;
            enter(LIDAR_DMA_FAILED);
        }
        break;
    case LIDAR_DMA_RETRY_DELAY:
    {
        /* Keep retrying after the fast attempts, with a longer gap between commands. */
        const uint32_t delay_ms = attempt < FAST_ATTEMPTS ? RETRY_DELAY_MS : RETRY_COOLDOWN_MS;
        if (elapsed(state_started_ms, delay_ms))
        {
            if (attempt <= FAST_ATTEMPTS)
                ++attempt;
            ++stats.retries;
            enter(LIDAR_DMA_SEND_SYNC);
        }
        break;
    }
    case LIDAR_DMA_IDLE:
    case LIDAR_DMA_FAILED:
        break;
    }
}

const volatile LidarDma_Stats *LidarDma_GetStats(void)
{
    return &stats;
}

/* Member 2 passes &huart1 as argument; board setup and queue creation happen
   before this thread starts. Timed blocking also detects a silent sensor. */
static _Noreturn void suspend_failed_task(void)
{
    /* W4Threads retains this handle for diagnostic stack sampling. */
    for(;;)(void)osThreadSuspend(osThreadGetId());
}

void W4LidarTask(void *argument)
{
    UART_HandleTypeDef *huart = (argument != NULL) ? (UART_HandleTypeDef *)argument : &huart1;
    if (!W4Queues_Ready())
    {
        stats.state = LIDAR_DMA_FAILED;
        suspend_failed_task();
    }
    LidarParser_Init();
    LidarDma_Init(huart);
    if (!LidarDma_Start())
    {
        stats.state = LIDAR_DMA_FAILED;
        suspend_failed_task();
    }
    for (;;)
    {
        if (stats.state == LIDAR_DMA_FAILED)
        {
            suspend_failed_task();
        }
        if (stats.state == LIDAR_DMA_STREAM)
        {
            W4LidarBlock block;
            const osStatus_t status = W4Queue_LidarGet(&block, 20U);
            W4Runtime_Begin(W4_TASK_LIDAR,osKernelGetTickCount());
            LidarDma_Poll(); /* checks timeout/UART fault even when queue is idle */
            if (status == osOK) LidarDma_ProcessBlock(&block);
            else if (status != osErrorTimeout && status != osErrorResource)
            {
                ++stats.queue_errors;
                recover();
                enter(LIDAR_DMA_FAILED);
            }
            W4Runtime_End(W4_TASK_LIDAR,0);
        }
        else
        {
            W4Runtime_Begin(W4_TASK_LIDAR,osKernelGetTickCount());
            LidarDma_Poll();
            W4Runtime_End(W4_TASK_LIDAR,0);
            /* The 10 s handshake and retry deadlines advance without spinning. */
            if (stats.state != LIDAR_DMA_STREAM) (void)osDelay(2U);
        }
    }
}
