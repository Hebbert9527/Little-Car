#include "lidar_parser.h"

#include <stdbool.h>
#include <string.h>

enum
{
    SCAN_RECORD_SIZE = 5,
    SCAN_START_BIT = 0x01,
    SCAN_INVERSE_START_BIT = 0x02,
    SCAN_CHECK_BIT = 0x01,
    SCAN_QUALITY_SHIFT = 2,
    SCAN_ANGLE_Q6_LIMIT = 360 * 64
};

/* Implemented by the application and called synchronously from Feed. */
void App_OnLidarPoint(float angle_deg, float distance_mm, uint8_t quality, bool new_scan);

static uint8_t s_pending[SCAN_RECORD_SIZE];
static size_t s_pending_len;
static bool s_scan_start_pending;
static LidarParser_Stats s_stats;

static uint16_t ReadLe16(const uint8_t *bytes)
{
    return (uint16_t)(bytes[0] | ((uint16_t)bytes[1] << 8));
}

static void DiscardFirstByte(void)
{
    /* The other four bytes can still belong to the next record. */
    memmove(s_pending, s_pending + 1, SCAN_RECORD_SIZE - 1);
    s_pending_len = SCAN_RECORD_SIZE - 1;
    ++s_stats.resync_bytes_discarded;
}

void LidarParser_Init(void)
{
    s_pending_len = 0;
    s_scan_start_pending=false;
    s_stats = (LidarParser_Stats){0};
}

void LidarParser_ResetStream(void)
{
    s_pending_len = 0;
    s_scan_start_pending=false;
    ++s_stats.stream_resets;
}

void LidarParser_Feed(const uint8_t *data, size_t len)
{
    if (data == NULL)
    {
        return;
    }

    for (size_t i = 0; i < len; ++i)
    {
        s_pending[s_pending_len++] = data[i];
        if (s_pending_len < SCAN_RECORD_SIZE)
        {
            continue;
        }

        const uint8_t flags = s_pending[0];
        const bool new_scan = (flags & SCAN_START_BIT) != 0;
        const bool inverse_start = (flags & SCAN_INVERSE_START_BIT) != 0;
        if (new_scan == inverse_start || (s_pending[1] & SCAN_CHECK_BIT) == 0)
        {
            ++s_stats.bad_header_windows;
            DiscardFirstByte();
            continue;
        }

        const uint16_t angle_q6 = ReadLe16(s_pending + 1) >> 1;
        if (angle_q6 >= SCAN_ANGLE_Q6_LIMIT)
        {
            ++s_stats.bad_angle_windows;
            DiscardFirstByte();
            continue;
        }

        const uint16_t distance_q2 = ReadLe16(s_pending + 3);
        s_pending_len = 0;
        s_scan_start_pending |= new_scan;
        if (distance_q2 == 0)
        {
            /* A no-return measurement still occupies a complete record. */
            ++s_stats.zero_distance_records;
            continue;
        }

        const uint8_t quality = flags >> SCAN_QUALITY_SHIFT;
        ++s_stats.points_emitted;
        App_OnLidarPoint(angle_q6 / 64.0f, distance_q2 / 4.0f, quality, s_scan_start_pending);
        s_scan_start_pending=false;
    }
}

LidarParser_Stats LidarParser_GetStats(void)
{
    return s_stats;
}
