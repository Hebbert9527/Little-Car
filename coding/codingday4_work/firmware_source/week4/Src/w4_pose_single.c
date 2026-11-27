/* Wheel odometry and IMU heading fusion. Older single-channel builds infer
 * the left wheel from the right encoder and IMU; dual builds measure both. */
#include "w4_nav.h"
#include <math.h>

#ifdef W4_DUAL_ENCODER
#include "w4_wheel_speed.h"

bool W4Encoder_TakeDelta(W4EncoderDelta *out) {
    static uint16_t previous_left, previous_right;
    static bool have_previous;
    if (!out) return false;
    uint16_t left, right;
    bool ready = W4Port_ReadEncoderCounts(&left, &right);
    *out = (W4EncoderDelta){.stamp_ms = W4Port_Millis(), .valid = ready};
    if (ready) {
        if (have_previous) {
            int32_t left_delta = W4WheelSpeed_CountDelta(left, previous_left);
            int32_t right_delta = W4WheelSpeed_CountDelta(right, previous_right);
#ifdef W4_REAR_FORWARD
            /* Calibration applies the reversed sign to each logical wheel. */
            out->left_delta = right_delta;
            out->right_delta = left_delta;
#else
            out->left_delta = left_delta;
            out->right_delta = right_delta;
#endif
        }
        previous_left = left; previous_right = right;
    }
    have_previous = ready;
    return ready;
}
#else
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
#endif

bool W4Pose_Update(uint32_t now, const W4EncoderDelta *e, float dt, float *vl, float *vr) {
    W4ImuSample i;
    bool have_imu=W4Imu_GetSnapshot(&i);
    /* Direct callers can also capture their release time before a new sensor
       publication. Check ages after both snapshots in the same HAL domain. */
    now=W4Port_Millis();
#if defined(W4_MAZE) && defined(W4_DUAL_ENCODER)
    static bool imu_fallback;
    static uint8_t still_ticks;
    if (!e->valid || !W4HardwareReady()) return false;
    if (!isfinite(dt) || dt <= 0.0f || dt > 0.1f) dt = 0.01f;
    bool imu_ok = have_imu && i.valid && i.calibrated && now - i.stamp_ms <= 200U &&
                  isfinite(i.heading_rad) && isfinite(i.gz_rad_s);
    float dr = (float)e->right_delta * w4_config.encoder_sign_r * W4_PI *
               w4_config.diameter_r_m / w4_config.cpr_r;
    float dl = (float)e->left_delta * w4_config.encoder_sign_l * W4_PI *
               w4_config.diameter_l_m / w4_config.cpr_l;
    *vr = dr / dt; *vl = dl / dt;
    if (!isfinite(*vl) || !isfinite(*vr) || fabsf(*vl) > 2.5f || fabsf(*vr) > 2.5f) return false;
    W4Pose p; W4Pose_GetSnapshot(&p);
    if (!p.valid) { still_ticks = 0; return false; } /* reset explicitly at physical start */
    if (e->left_delta == 0 && e->right_delta == 0) {
        if (still_ticks < 20U) ++still_ticks;
    } else {
        still_ticks = 0;
    }
    float ds = 0.5f * (dl + dr);
    float da;
    if (imu_ok) {
        if (imu_fallback) {
            /* Re-anchor IMU offset to encoder-integrated heading so recovery never jumps. */
            w4_config.imu_offset_rad = W4Wrap(p.theta_rad - i.heading_rad * w4_config.imu_sign);
            imu_fallback = false;
        }
        /* ZUPT: when both wheels have zero counts for >=50 ms and gyro rate is
           within stationary thermal drift (<1.2 deg/s), hold heading and absorb
           any accumulated gyro bias drift so parked scans never smear walls. */
        if (still_ticks >= 5U && fabsf(i.gz_rad_s) < (1.2f * W4_PI / 180.0f)) {
            w4_config.imu_offset_rad = W4Wrap(p.theta_rad - i.heading_rad * w4_config.imu_sign);
        } else {
            float yaw = i.gz_rad_s * w4_config.imu_sign;
            da = yaw * dt;
            float mid = p.theta_rad + da * 0.5f;
            p.x_m += ds * cosf(mid); p.y_m += ds * sinf(mid);
            float predicted = W4Wrap(p.theta_rad + da);
            float heading = W4Wrap(i.heading_rad * w4_config.imu_sign + w4_config.imu_offset_rad);
            p.theta_rad = W4Wrap(predicted + (dt / (0.5f + dt)) * W4Wrap(heading - predicted));
        }
    } else {
        /* Seamless dual-encoder dead-reckoning during transient I2C/IMU glitches. */
        imu_fallback = true;
        da = (dr - dl) / w4_config.track_m;
        float mid = p.theta_rad + da * 0.5f;
        p.x_m += ds * cosf(mid); p.y_m += ds * sinf(mid);
        p.theta_rad = W4Wrap(p.theta_rad + da);
    }
    p.stamp_ms = now;
    if (!isfinite(p.x_m) || !isfinite(p.y_m) || !isfinite(p.theta_rad) ||
        fabsf(p.x_m) > 5.3f || fabsf(p.y_m) > 5.3f) return false;
    uint32_t k = W4Port_Lock(); w4_nav.pose = p; W4Port_Unlock(k);
    return true;
#else
    if (!e->valid || now - e->stamp_ms > 30 || !W4HardwareReady() || dt <= 0 || dt > 0.03f ||
        !have_imu || !i.valid || !i.calibrated || now - i.stamp_ms > 50 ||
        !isfinite(i.heading_rad) || !isfinite(i.gz_rad_s)) return false;
    float dr = (float)e->right_delta * w4_config.encoder_sign_r * W4_PI *
               w4_config.diameter_r_m / w4_config.cpr_r;
    float yaw = i.gz_rad_s * w4_config.imu_sign;
#ifdef W4_DUAL_ENCODER
    float dl = (float)e->left_delta * w4_config.encoder_sign_l * W4_PI *
               w4_config.diameter_l_m / w4_config.cpr_l;
#else
    float dl = dr - yaw * w4_config.track_m * dt;
#endif
    *vr = dr / dt; *vl = dl / dt;
    if (fabsf(*vl) > 1.5f || fabsf(*vr) > 1.5f) return false;
    W4Pose p; W4Pose_GetSnapshot(&p);
    if (!p.valid) return false; /* reset explicitly at physical start */
    float ds = 0.5f * (dl + dr);
    /* Gyro heading keeps turn estimation independent of wheel slip. */
    float da = yaw * dt;
    float mid = p.theta_rad + da * 0.5f;
    p.x_m += ds * cosf(mid); p.y_m += ds * sinf(mid);
    float predicted = W4Wrap(p.theta_rad + da);
    float heading = W4Wrap(i.heading_rad * w4_config.imu_sign + w4_config.imu_offset_rad);
    /* dt-based gain: a repeated IMU sample must not get a fixed per-call weight. */
    p.theta_rad = W4Wrap(predicted + (dt / (0.5f + dt)) * W4Wrap(heading - predicted));
    p.stamp_ms = now;
    if (!isfinite(p.x_m) || !isfinite(p.y_m) || !isfinite(p.theta_rad) ||
        fabsf(p.x_m) > 5.3f || fabsf(p.y_m) > 5.3f) return false;
    uint32_t k = W4Port_Lock(); w4_nav.pose = p; W4Port_Unlock(k);
    return true;
#endif
}
