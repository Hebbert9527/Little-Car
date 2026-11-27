/* Reviewed single-encoder dead reckoning (build flag W4_SINGLE_ENCODER).
 *
 * HARDWARE HONESTY: the board has ONE unsigned encoder on PA7 (TIM3 input
 * capture, right wheel). It has no direction and no left channel. This file
 * replaces member6's dual-wheel w4_pose.c and is NOT dual quadrature odometry:
 *   - right distance = PA7 edge rate * RIGHT drive sign (w4_single_cmd_dir)
 *   - heading   = IMU gyro integration fused with the calibrated IMU heading
 *   - centre velocity = right velocity - gyro yaw rate * track / 2;
 *     left velocity = right velocity - gyro yaw rate * track (an estimate).
 * Known limits (must be stated in the demo): coasting with motors stopped is
 * assigned the previous direction for at most 200 ms; ambiguous reversal
 * intervals and later unpowered edges invalidate the sample. Slip degrades accuracy;
 * turn odometry relies on the IMU, not wheel differential.
 * Motion stays locked until w4_config.verified/encoders_verified/imu_axes_verified
 * are set after physical calibration (see W4HardwareReady).
 */
#include "w4_nav.h"
#include <math.h>

/* ISR-maintained rising-edge counter (app_state.inc / app_encoder.inc).
   330 rising edges per output revolution by default; set w4_config.cpr_r to the
   measured value. Read under W4Port_Lock (PRIMASK covers the priority-0 TIM3 ISR). */
extern volatile uint32_t encoder_edge_count;
extern int w4_single_cmd_dir; /* RIGHT commanded drive sign, from w4_port_stm32.c */

bool W4Encoder_TakeDelta(W4EncoderDelta *o) {
    if (!o) return false;
    static uint32_t previous;
    static bool baseline;
    static int last_dir;
    static uint32_t powered_ms;
    static uint32_t last_edges_ms;
    uint32_t k = W4Port_Lock();
    uint32_t edges = encoder_edge_count;
    int dir = w4_single_cmd_dir;
    W4Port_Unlock(k);
    int32_t delta = baseline ? W4CountDelta(edges, previous) : 0;
    previous = edges; baseline = true;
    o->stamp_ms = W4Port_Millis();
    bool ambiguous=false;
    if(dir) {
        /* Counts spanning a sign change have unknown direction. Never assign
           the whole interval the new sign. The controller stops on invalidity. */
        if(last_dir && dir!=last_dir && o->stamp_ms-last_edges_ms<50) ambiguous=true;
        last_dir=dir; powered_ms=o->stamp_ms;
    } else if(delta && (!last_dir || o->stamp_ms-powered_ms>200)) ambiguous=true;
    if(delta) last_edges_ms=o->stamp_ms;
    o->left_delta=0; /* no physical left encoder */
    o->right_delta=ambiguous?0:last_dir*delta;
    o->valid = !ambiguous && delta>=0 && w4_config.encoders_verified && W4Port_EncodersReady();
    return o->valid != 0;
}

bool W4Pose_Update(uint32_t now, const W4EncoderDelta *e, float dt, float *vl, float *vr) {
    W4ImuSample i;
    bool have_imu=W4Imu_GetSnapshot(&i);
    /* Direct callers can also capture their release time before a new sensor
       publication. Check ages after both snapshots in the same HAL domain. */
    now=W4Port_Millis();
    if (!e->valid || now - e->stamp_ms > 30 || !W4HardwareReady() || dt <= 0 || dt > 0.03f ||
        !have_imu || !i.valid || !i.calibrated || now - i.stamp_ms > 50 ||
        !isfinite(i.heading_rad) || !isfinite(i.gz_rad_s)) return false;
    /* Only right displacement is measured. Gyro supplies the turn component. */
    float dr = (float)e->right_delta * w4_config.encoder_sign_r * W4_PI *
               w4_config.diameter_r_m / w4_config.cpr_r;
    float yaw=i.gz_rad_s*w4_config.imu_sign;
    *vr = dr / dt; *vl = *vr-yaw*w4_config.track_m;
    if (fabsf(*vl) > 1.5f || fabsf(*vr) > 1.5f) return false;
    W4Pose p; W4Pose_GetSnapshot(&p);
    if (!p.valid) return false; /* reset explicitly at physical start */
    float ds = dr-yaw*w4_config.track_m*0.5f*dt;
    /* Heading change from the gyro (da), since wheel differential is unavailable. */
    float da = yaw * dt;
    float mid = p.theta_rad + da * 0.5f;
    p.x_m += ds * cosf(mid); p.y_m += ds * sinf(mid);
    float predicted = W4Wrap(p.theta_rad + da);
    float heading = W4Wrap(i.heading_rad * w4_config.imu_sign + w4_config.imu_offset_rad);
    /* dt-based gain: a repeated IMU sample must not get a fixed per-call weight. */
    p.theta_rad = W4Wrap(predicted + (dt / (0.5f + dt)) * W4Wrap(heading - predicted));
    p.stamp_ms = now;
    if (!isfinite(p.x_m) || !isfinite(p.y_m) || !isfinite(p.theta_rad) ||
        hypotf(p.x_m, p.y_m) > 5) return false;
    uint32_t k = W4Port_Lock(); w4_nav.pose = p; W4Port_Unlock(k);
    return true;
}
