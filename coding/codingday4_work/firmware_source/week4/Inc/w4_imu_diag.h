#ifndef W4_IMU_DIAG_H
#define W4_IMU_DIAG_H
#include <stdint.h>
#define W4_IMU_DIAG_SAMPLES 500U
/* Diagnostic build only. Frozen statistics update each 500 decoded task samples.
 * ready is a publication sequence: odd while copying, even when complete.
 * Config blocks: 0x13..0x1c, PWR_MGMT_1/2, WHO_AM_I, then three pad bytes.
 * Float channels: ax, ay, az (g), gx, gy, gz (degrees/s), temperature raw.
 * read_mask bits 0..2 are before setup, 3..5 after setup. */
typedef struct {
    uint32_t magic, configurations, read_mask, samples, ready, first_ms, last_ms;
    uint8_t before[16], after[16];
    float mean[7], m2[7], minimum[7], maximum[7];
} W4ImuDiagnostic;
extern volatile W4ImuDiagnostic W4ImuDiag;
#endif
