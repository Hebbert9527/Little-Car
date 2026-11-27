/* Geometry and measured encoder ratios. Define W4_CALIBRATED only after
 * physical checks of the wheels, IMU axes and navigation drive. */
#include "w4_nav.h"

#ifndef W4_CAL_DIAMETER_R_M
#define W4_CAL_DIAMETER_R_M   0.068f          /* 2026-11-18: user measured 68 mm tire OD */
#endif
#ifndef W4_CAL_CPR_R
#ifdef W4_DUAL_ENCODER
#define W4_CAL_CPR_R          1561.60f       /* 2026-11-19: 15616 x4 counts / 10 turns */
#else
#define W4_CAL_CPR_R          391.15f        /* Rising edges per right wheel revolution */
#endif
#endif
#ifndef W4_CAL_CPR_L
#ifdef W4_DUAL_ENCODER
#define W4_CAL_CPR_L          1559.45f       /* 2026-11-19: 31189 x4 counts / 20 turns */
#else
#define W4_CAL_CPR_L          W4_CAL_CPR_R
#endif
#endif
#ifndef W4_CAL_ENC_SIGN_L
#define W4_CAL_ENC_SIGN_L     1
#endif
#ifndef W4_CAL_TRACK_M
#define W4_CAL_TRACK_M        0.175f          /* 200 mm outer span - two 25 mm half-widths */
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
    w4_config.track_m        = W4_CAL_TRACK_M;
#ifdef W4_REAR_FORWARD
    w4_config.cpr_r          = W4_CAL_CPR_L;
    w4_config.cpr_l          = W4_CAL_CPR_R;
    w4_config.encoder_sign_r = -W4_CAL_ENC_SIGN_L;
    w4_config.encoder_sign_l = -W4_CAL_ENC_SIGN_R;
#else
    w4_config.cpr_r          = W4_CAL_CPR_R;
    w4_config.cpr_l          = W4_CAL_CPR_L;
    w4_config.encoder_sign_r = W4_CAL_ENC_SIGN_R;
    w4_config.encoder_sign_l = W4_CAL_ENC_SIGN_L;
#endif
    w4_config.imu_sign       = W4_CAL_IMU_SIGN;
    w4_config.imu_offset_rad = W4_CAL_IMU_OFFSET_RAD;
#ifdef W4_CALIBRATED
    w4_config.verified          = true;
    w4_config.encoders_verified = true;
    w4_config.imu_axes_verified = true;
#endif
}