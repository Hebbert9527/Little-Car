#ifndef W4_NAV_H
#define W4_NAV_H
#include "w4_contract.h"
#define W4_PI 3.14159265358979323846f
#if defined(W4_REAR_FORWARD) && !defined(W4_DUAL_ENCODER)
#error "Rear-forward orientation requires independent feedback from both wheels."
#endif
typedef enum { W4_IDLE, W4_PLANNING, W4_TRACKING, W4_ARRIVED, W4_STOPPED, W4_FAULT } W4State;
typedef struct {
    /* Zero defaults intentionally prevent motion. Set only after calibration. */
    bool verified, encoders_verified, imu_axes_verified;
    float diameter_l_m, diameter_r_m, track_m, cpr_l, cpr_r;
    int encoder_sign_l, encoder_sign_r, imu_sign;
    float imu_offset_rad; /* old IMU zero -> world +pi/2 */
    float wheel_kp, wheel_ki, wheel_kd, wheel_ff, pwm_deadzone;
    float heading_kp, heading_ki, heading_kd;
    float max_v, max_w, max_wheel, pwm_limit;
    float position_tol, heading_tol, step_m;
    bool diagram_positive_route_confirmed;
} W4Config;
extern W4Config w4_config; /* configure before tasks; only stopped reset changes heading offset */
typedef struct { float integral, previous, derivative; bool have_previous; } W4Pid;
typedef struct {
    uint32_t rx_overflow, rx_long, rx_invalid, rx_errors, ack_lost;
    uint32_t cmd_full, path_wait, stale_wp, deadline_miss;
    uint32_t pose_us, tracking_us, motor_us, cycle_us, max_cycle_us;
} W4Stats;
typedef struct {
    W4State state; uint32_t cmd_id; float target_x,target_y;
    uint16_t wp_index; uint32_t fault; W4Pose pose;
} W4NavSnapshot;
void W4Nav_Init(void); /* before threads; PWM initialized in safe state */
bool W4Nav_ResetPose(float x,float y,float theta); /* stopped physical reset */
bool W4Nav_GetSnapshot(W4NavSnapshot *out);
/* Member 1 TX task calls at its 50 ms publish period; separate N sequence. */
uint16_t W4Nav_FormatTelemetry(char *out,uint16_t capacity,uint32_t now_ms);
void W4Nav_GetStats(W4Stats *out);
void W4Bt_OnByteISR(uint8_t byte); /* USART3 only, never LiDAR callback */
void W4Bt_OnErrorISR(void);
void W4Bt_Process(void); /* also used by host tests */
void W4Path_Poll(void);
void W4Control_Step(uint32_t now_ms);
void W4Encoder_OnEdgeISR(unsigned side,int direction); /* 0 left,1 right; measured +/-1 */
float W4Wrap(float angle);
int32_t W4CountDelta(uint32_t now,uint32_t before);
bool W4ParseCommand(const char *line,W4Command *out);
float W4Pid_Update(W4Pid *p,float error,float dt,float kp,float ki,float kd,float limit);
void W4Pid_Reset(W4Pid *p);
/* Port hooks. Short interrupt mask MUST cover priority-0 encoder ISR too.
   No mutex/RTOS calls inside these critical sections. */
uint32_t W4Port_Lock(void); void W4Port_Unlock(uint32_t key);
uint32_t W4Port_Millis(void); uint32_t W4Port_Micros(void);
bool W4Port_EncodersReady(void); /* configured physical encoders available */
#ifdef W4_DUAL_ENCODER
bool W4Port_ReadEncoderCounts(uint16_t *left, uint16_t *right);
#endif
void W4Port_WritePwm(float left,float right); /* signed logical +/-1, apply polarity here */
bool W4Port_RxStart(void); /* task context; receive 1 byte IT */
bool W4Port_RxRecover(void); /* RX-only recovery, preserve TX DMA */
/* Private path/control marker. For this item y_m carries a forward distance. */
enum {
    W4_WAYPOINT_INCOMING_LEG = 1U << 11,
    W4_WAYPOINT_AXIS_X = 1U << 12,
    W4_WAYPOINT_AXIS_Y = 1U << 13,
    W4_WAYPOINT_NEGATIVE = 1U << 14,
    W4_CORNER_WAYPOINT = 1U << 15,
    W4_WAYPOINT_INDEX_MASK = W4_WAYPOINT_INCOMING_LEG - 1U,
    W4_DISTANCE_WAYPOINT = UINT16_MAX
};
/* Private shared functions; only task owners call them. */
void W4Ack(uint32_t id,const char *status,const char *reason);
bool W4HardwareReady(void);
bool W4Pose_Update(uint32_t now,const W4EncoderDelta *e,float dt,float *vl,float *vr);
void W4Motor_Update(float vl,float vr,float dt,uint32_t now,bool fixed_distance);
#if defined(W4_DUAL_ENCODER) && defined(W4_WHEEL_DIAGNOSTICS)
uint16_t W4Motor_FormatDiagnostics(char *out, uint16_t capacity);
#endif
extern W4NavSnapshot w4_nav;
#ifdef W4_REAR_FORWARD
typedef struct { bool enabled; float base[2]; } W4StraightCalibration;
extern W4StraightCalibration w4_straight_cal;
extern volatile float w4_cruise_request; /* m/s, zero uses route defaults */
bool W4Control_CalibrationReady(uint32_t now); /* caller holds port lock */
#endif
extern W4Stats w4_stats;
extern bool w4_stop_pending;
extern bool w4_reset_pending;
extern uint32_t w4_stop_epoch,w4_control_stop_epoch;
#endif
