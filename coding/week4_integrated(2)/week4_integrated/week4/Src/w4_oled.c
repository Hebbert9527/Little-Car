#include "w4_oled.h"
#include "w4_i2c_bus.h"
#include "w4_runtime.h"

static W4OledStats stats = {
    .status = W4_OLED_CONTROLLER == W4_OLED_NONE ? W4_OLED_UNCONFIGURED : W4_OLED_STARTING
};

void W4Oled_GetStats(W4OledStats *out)
{
    if (out != NULL) {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        *out = stats;
        __set_PRIMASK(mask);
    }
}

#if W4_OLED_CONTROLLER == W4_OLED_SSD1306

#include <stdio.h>
#include <string.h>

#if !defined(W4_OLED_ADDR7) || !defined(W4_OLED_WIDTH) || !defined(W4_OLED_HEIGHT)
#error "Set W4_OLED_ADDR7, W4_OLED_WIDTH and W4_OLED_HEIGHT for the confirmed module"
#endif
#if (W4_OLED_ADDR7 != 0x3c && W4_OLED_ADDR7 != 0x3d) || W4_OLED_WIDTH != 128 || \
    (W4_OLED_HEIGHT != 32 && W4_OLED_HEIGHT != 64)
#error "Supported SSD1306 modules use 128x32 or 128x64 and address 0x3c or 0x3d"
#endif

#define PERIOD_MS 50U
#define TRANSFER_BUDGET_MS 8U
#define CHUNK_SIZE 16U
#define ROWS 4U
#define CHUNKS_PER_PAGE (W4_OLED_WIDTH / CHUNK_SIZE)
#define SCREEN_CHUNKS (W4_OLED_HEIGHT / 8U * CHUNKS_PER_PAGE)
#define DASHBOARD_CHUNKS (ROWS * CHUNKS_PER_PAGE)

static uint8_t desired[ROWS][W4_OLED_WIDTH];
static uint8_t displayed[ROWS][W4_OLED_WIDTH];
static const uint8_t blank[CHUNK_SIZE];

static const uint8_t setup_display[] = {
    0x00,                         /* Co=0, command stream */
    0xae, 0xd5, 0x80,            /* off, clock */
    0xa8, W4_OLED_HEIGHT - 1U,    /* multiplex ratio */
    0xd3, 0x00, 0x40,            /* offset and start line */
    0x8d, 0x14,                  /* internal charge pump */
    0x20, 0x02,                  /* page addressing */
    0xa1, 0xc8                   /* segment and COM scan direction */
};
static const uint8_t setup_panel[] = {
    0x00, 0xda, W4_OLED_HEIGHT == 64 ? 0x12 : 0x02,
    0x81, 0x7f, 0xd9, 0xf1, 0xdb, 0x30, 0xa4, 0xa6, 0x2e
};
static const uint8_t enable_display[] = {0x00, 0xaf};

static const uint8_t letters[26][5] = {
    {0x7e,0x11,0x11,0x11,0x7e}, {0x7f,0x49,0x49,0x49,0x36},
    {0x3e,0x41,0x41,0x41,0x22}, {0x7f,0x41,0x41,0x22,0x1c},
    {0x7f,0x49,0x49,0x49,0x41}, {0x7f,0x09,0x09,0x09,0x01},
    {0x3e,0x41,0x49,0x49,0x7a}, {0x7f,0x08,0x08,0x08,0x7f},
    {0x00,0x41,0x7f,0x41,0x00}, {0x20,0x40,0x41,0x3f,0x01},
    {0x7f,0x08,0x14,0x22,0x41}, {0x7f,0x40,0x40,0x40,0x40},
    {0x7f,0x02,0x0c,0x02,0x7f}, {0x7f,0x04,0x08,0x10,0x7f},
    {0x3e,0x41,0x41,0x41,0x3e}, {0x7f,0x09,0x09,0x09,0x06},
    {0x3e,0x41,0x51,0x21,0x5e}, {0x7f,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7f,0x01,0x01},
    {0x3f,0x40,0x40,0x40,0x3f}, {0x1f,0x20,0x40,0x20,0x1f},
    {0x3f,0x40,0x38,0x40,0x3f}, {0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}
};
static const uint8_t digits[10][5] = {
    {0x3e,0x51,0x49,0x45,0x3e}, {0,0x42,0x7f,0x40,0},
    {0x62,0x51,0x49,0x49,0x46}, {0x22,0x41,0x49,0x49,0x36},
    {0x18,0x14,0x12,0x7f,0x10}, {0x2f,0x49,0x49,0x49,0x31},
    {0x3e,0x49,0x49,0x49,0x32}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x26,0x49,0x49,0x49,0x3e}
};

static const uint8_t *glyph(char c)
{
    static const uint8_t plus[5] = {0x08,0x08,0x3e,0x08,0x08};
    static const uint8_t minus[5] = {0x08,0x08,0x08,0x08,0x08};
    if (c >= 'A' && c <= 'Z')
        return letters[c - 'A'];
    if (c >= '0' && c <= '9')
        return digits[c - '0'];
    if (c == '+')
        return plus;
    if (c == '-')
        return minus;
    return blank;
}

static void render_line(unsigned row, const char *text)
{
    memset(desired[row], 0, W4_OLED_WIDTH);
    for (unsigned x = 0; *text != '\0' && x + 5U < W4_OLED_WIDTH; x += 6U)
        memcpy(&desired[row][x], glyph(*text++), 5U);
}

static void render_dashboard(void)
{
    static const char *const states[] = {
        "IDLE", "PLANNING", "TRACKING", "ARRIVED", "STOPPED", "FAULT"
    };
    W4OledNavSnapshot nav = {0};
    bool valid = W4Oled_ReadNavSnapshot(&nav);
    char text[32];
    if (valid && nav.pose_valid) {
        snprintf(text, sizeof text, "XY %+ld %+ld CM", (long)(nav.x_mm / 10),
                 (long)(nav.y_mm / 10));
        render_line(0, text);
        snprintf(text, sizeof text, "HDG %+ld MRAD", (long)nav.theta_mrad);
        render_line(1, text);
    } else {
        render_line(0, "XY --");
        render_line(1, "HDG --");
    }
    if (valid && nav.target_valid) {
        snprintf(text, sizeof text, "GO %+ld %+ld CM", (long)(nav.target_x_mm / 10),
                 (long)(nav.target_y_mm / 10));
        render_line(2, text);
    } else {
        render_line(2, "GO --");
    }
    unsigned state = (unsigned)nav.state;
    if (valid && state < sizeof states / sizeof states[0]) {
        snprintf(text, sizeof text, "%s F %lu", states[state], (unsigned long)nav.fault);
        render_line(3, text);
    } else {
        render_line(3, "NAV --");
    }
}

static bool write_bytes(const uint8_t *buf, uint16_t len)
{
    return W4I2c_Write(W4_OLED_ADDR7, buf, len, TRANSFER_BUDGET_MS);
}

static bool write_chunk(unsigned index, const uint8_t *pixels)
{
    unsigned page = index / CHUNKS_PER_PAGE;
    unsigned col = index % CHUNKS_PER_PAGE * CHUNK_SIZE;
    /* Co=1 before each positioning command, then Co=0/D-C=1 for pixels. */
    uint8_t frame[7U + CHUNK_SIZE] = {
        0x80, (uint8_t)(0xb0U | page), 0x80, (uint8_t)(col & 0x0fU),
        0x80, (uint8_t)(0x10U | (col >> 4U)), 0x40
    };
    memcpy(&frame[7], pixels, CHUNK_SIZE);
    return write_bytes(frame, sizeof frame);
}

static bool refresh(unsigned *cursor, uint32_t *chunks_sent)
{
    render_dashboard();
    for (unsigned checked = 0; checked < DASHBOARD_CHUNKS; ++checked) {
        unsigned index = *cursor;
        unsigned page = index / CHUNKS_PER_PAGE;
        unsigned col = (index % CHUNKS_PER_PAGE) * CHUNK_SIZE;
        *cursor = (index + 1U) % DASHBOARD_CHUNKS;
        if (memcmp(&desired[page][col], &displayed[page][col], CHUNK_SIZE) != 0) {
            if (!write_chunk(index, &desired[page][col]))
                return false;
            memcpy(&displayed[page][col], &desired[page][col], CHUNK_SIZE);
            ++*chunks_sent;
            return true;
        }
    }
    /* Bounded refresh even when the robot is stationary: both real devices
       continue using I2C1 so the mutex demo has sustained, observable traffic. */
    unsigned index=*cursor;
    unsigned page=index/CHUNKS_PER_PAGE,col=(index%CHUNKS_PER_PAGE)*CHUNK_SIZE;
    if(!write_chunk(index,&desired[page][col]))return false;
    ++*chunks_sent;
    *cursor=(index+1U)%DASHBOARD_CHUNKS;
    return true;
}

static void publish(const W4OledStats *current)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    stats = *current;
    __set_PRIMASK(mask);
}

void W4OledTask(void *argument)
{
    (void)argument;
    const uint32_t period = (osKernelGetTickFreq() * PERIOD_MS + 999U) / 1000U;
    uint32_t next = osKernelGetTickCount();
    unsigned stage = 0, cursor = 0, failures = 0;
    W4OledStats current = {.status = W4_OLED_STARTING};
    for (;;) {
        W4Runtime_Begin(W4_TASK_OLED,next);
        bool ok;
        ++current.periods;
        if (stage == 0) {
            ok = write_bytes(setup_display, sizeof setup_display);
        } else if (stage == 1) {
            ok = write_bytes(setup_panel, sizeof setup_panel);
        } else if (stage < SCREEN_CHUNKS + 2U) {
            ok = write_chunk(stage - 2U, blank);
            if (ok)
                ++current.chunks_sent;
        } else if (stage == SCREEN_CHUNKS + 2U) {
            ok = write_bytes(enable_display, sizeof enable_display);
            if (ok)
                memset(displayed, 0, sizeof displayed);
        } else {
            ok = refresh(&cursor, &current.chunks_sent);
        }
        if (ok) {
            failures = 0;
            if (stage < SCREEN_CHUNKS + 3U)
                ++stage;
            current.status = stage == SCREEN_CHUNKS + 3U ? W4_OLED_READY : W4_OLED_STARTING;
        } else {
            ++current.io_failures;
            current.status = W4_OLED_IO_ERROR;
            if (++failures == 3U) {
                stage = 0;
                failures = 0;
            }
        }

        next += period;
        uint32_t late = osKernelGetTickCount() - next;
        if (late <= INT32_MAX) {
            uint32_t missed = late / period + 1U;
            current.missed_periods += missed;
            next += missed * period;
        }
        publish(&current);
        W4Runtime_End(W4_TASK_OLED,PERIOD_MS);
        (void)osDelayUntil(next);
    }
}

#elif W4_OLED_CONTROLLER == W4_OLED_NONE

void W4OledTask(void *argument)
{
    (void)argument;
    /* Preserve the TCB while W4Threads is used by the stack collector. */
    for(;;)(void)osThreadSuspend(osThreadGetId());
}

#else
#error "Unsupported OLED controller"
#endif
