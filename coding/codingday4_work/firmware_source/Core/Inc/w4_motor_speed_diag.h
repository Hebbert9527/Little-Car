#ifndef W4_MOTOR_SPEED_DIAG_H
#define W4_MOTOR_SPEED_DIAG_H
#include <stdint.h>

#define W4_SPEED_TRACE_CAPACITY 100u
typedef struct {
    uint32_t elapsed_ms;
    int32_t left_delta, right_delta;
    float left_rpm, right_rpm;
    float left_pwm, right_pwm;
} W4SpeedTraceSample;
typedef struct {
    uint32_t version, active;
    float target_rpm;
    uint32_t samples, fault;
    W4SpeedTraceSample trace[W4_SPEED_TRACE_CAPACITY];
} W4MotorSpeedDiagnostic;
extern volatile W4MotorSpeedDiagnostic w4_motor_speed_diag;
void W4MotorSpeedDiag_Start(uint32_t target_rpm);
void W4MotorSpeedDiag_Stop(void);
void W4MotorSpeedDiag_Poll(void);
#endif
