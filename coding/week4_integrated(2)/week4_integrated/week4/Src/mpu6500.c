#include "mpu6500.h"

static int16_t ReadBe16(const uint8_t *bytes)
{
    return (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

void Mpu6500_Decode(const uint8_t frame[14], Mpu6500Sample *sample)
{
    /* ACCEL_CONFIG=0: +/-2 g; GYRO_CONFIG=0: +/-250 degrees/s.
       Block 3.3 slides 53-55: signed big-endian gyro / 131. */
    sample->ax_g = (float)ReadBe16(frame) / 16384.0f;
    sample->ay_g = (float)ReadBe16(frame + 2) / 16384.0f;
    sample->az_g = (float)ReadBe16(frame + 4) / 16384.0f;
    sample->gx_dps = (float)ReadBe16(frame + 8) / 131.0f;
    sample->gy_dps = (float)ReadBe16(frame + 10) / 131.0f;
    sample->gz_dps = (float)ReadBe16(frame + 12) / 131.0f;
}
