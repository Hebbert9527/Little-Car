#ifndef W4_OLED_H
#define W4_OLED_H

#include <stdbool.h>
#include <stdint.h>

#define W4_OLED_NONE 0
#define W4_OLED_SSD1306 1

/* Configurable assumption: SSD1306 128x64 at 0x3C; confirm the physical module. */
#ifndef W4_OLED_CONTROLLER
#define W4_OLED_CONTROLLER W4_OLED_SSD1306
#endif
#ifndef W4_OLED_ADDR7
#define W4_OLED_ADDR7 0x3c
#endif
#ifndef W4_OLED_WIDTH
#define W4_OLED_WIDTH 128
#endif
#ifndef W4_OLED_HEIGHT
#define W4_OLED_HEIGHT 64
#endif

typedef enum {
    W4_OLED_NAV_IDLE, W4_OLED_NAV_PLANNING, W4_OLED_NAV_TRACKING,
    W4_OLED_NAV_ARRIVED, W4_OLED_NAV_STOPPED, W4_OLED_NAV_FAULT
} W4OledNavState;

typedef struct {
    int32_t x_mm, y_mm, theta_mrad, target_x_mm, target_y_mm;
    W4OledNavState state;
    uint32_t fault;
    bool pose_valid, target_valid;
} W4OledNavSnapshot;

typedef enum {
    W4_OLED_UNCONFIGURED, W4_OLED_STARTING, W4_OLED_READY, W4_OLED_IO_ERROR
} W4OledStatus;

typedef struct {
    W4OledStatus status;
    uint32_t periods, missed_periods, chunks_sent, io_failures;
} W4OledStats;

/* Control/navigation owner supplies one coherent copy when hardware is enabled. */
bool W4Oled_ReadNavSnapshot(W4OledNavSnapshot *out);
void W4OledTask(void *argument);
void W4Oled_GetStats(W4OledStats *out);

#endif
