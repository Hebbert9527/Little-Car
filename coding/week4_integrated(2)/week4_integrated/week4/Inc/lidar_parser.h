#ifndef LIDAR_PARSER_H
#define LIDAR_PARSER_H

#include <stddef.h>
#include <stdint.h>

typedef struct
{
    uint32_t points_emitted;
    uint32_t bad_header_windows;
    uint32_t bad_angle_windows;
    uint32_t zero_distance_records;
    uint32_t resync_bytes_discarded;
    uint32_t stream_resets;
} LidarParser_Stats;

/* Start a new stream and clear the counters and any buffered bytes. */
void LidarParser_Init(void);

/* Drop a partial record after a discontinuity, preserving diagnostics. */
void LidarParser_ResetStream(void);

/* Feed Standard Scan payload bytes in order from the main loop. */
void LidarParser_Feed(const uint8_t *data, size_t len);

/* Read a diagnostic snapshot from the same context as Feed. */
LidarParser_Stats LidarParser_GetStats(void);

#endif
