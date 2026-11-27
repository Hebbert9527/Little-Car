/* Physical calibration interface for the single-encoder navigation scheme.
 *
 * MEASURE these on the actual robot, fill the macros below, then build with
 * -DW4_CALIBRATED to unlock motion. WITHOUT W4_CALIBRATED the verified flags
 * stay false and W4HardwareReady() keeps the motors locked (safe default).
 *
 * Single-encoder notes:
 *   - cpr_r = measured RISING edges per output revolution (PA7/TIM3; the Week3
 *     firmware used 330). cpr_l/diameter_l are set equal to the right wheel
 *     because the left wheel is estimated, not measured.
 *   - encoder_sign_r = +1 if forward drive produces positive distance, else -1.
 *   - imu_sign / imu_offset_rad map the IMU heading into the world frame
 *     (world +pi/2 at the IMU zero by default). Verify against a known heading.
 *   - track_m = centre-to-centre wheel track; used by the heading PID to split
 *     left/right wheel speeds (the inner wheel speed is estimated during turns).
 */
#include "w4_nav.h"

#ifndef W4_CAL_DIAMETER_R_M
#define W4_CAL_DIAMETER_R_M   0.065f          /* TODO: measure right wheel diameter (m) */
#endif
#ifndef W4_CAL_CPR_R
#define W4_CAL_CPR_R          330.0f          /* TODO: measure rising edges per output rev */
#endif
#ifndef W4_CAL_TRACK_M
#define W4_CAL_TRACK_M        0.120f          /* TODO: measure wheel track (m) */
#endif
#ifndef W4_CAL_ENC_SIGN_R
#define W4_CAL_ENC_SIGN_R     1               /* TODO: +1 or -1 after direction check */
#endif
#ifndef W4_CAL_IMU_SIGN
#define W4_CAL_IMU_SIGN       1               /* TODO: +1 or -1 after axis check */
#endif
#ifndef W4_CAL_IMU_OFFSET_RAD
#define W4_CAL_IMU_OFFSET_RAD (W4_PI / 2)     /* TODO: IMU zero -> world heading offset */
#endif

void W4Calibration_Apply(void)
{
    w4_config.diameter_r_m   = W4_CAL_DIAMETER_R_M;
    w4_config.diameter_l_m   = W4_CAL_DIAMETER_R_M;
    w4_config.cpr_r          = W4_CAL_CPR_R;
    w4_config.cpr_l          = W4_CAL_CPR_R;
    w4_config.track_m        = W4_CAL_TRACK_M;
    w4_config.encoder_sign_r = W4_CAL_ENC_SIGN_R;
    w4_config.encoder_sign_l = W4_CAL_ENC_SIGN_R;
    w4_config.imu_sign       = W4_CAL_IMU_SIGN;
    w4_config.imu_offset_rad = W4_CAL_IMU_OFFSET_RAD;
#ifdef W4_CALIBRATED
    w4_config.verified          = true;
    w4_config.encoders_verified = true;
    w4_config.imu_axes_verified = true;
#endif
}