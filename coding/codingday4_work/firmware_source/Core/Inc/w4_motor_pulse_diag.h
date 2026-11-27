#ifndef W4_MOTOR_PULSE_DIAG_H
#define W4_MOTOR_PULSE_DIAG_H
#include <stdint.h>

typedef struct {
    volatile uint32_t request;       /* 1 left, 2 right, 3 both when enabled */
    volatile uint32_t state;         /* 0 idle, 1 running, 2 stopped, 3 invalid */
    volatile uint32_t last_side;
    volatile uint32_t start_ms, stop_ms;
    volatile uint32_t left_start, left_stop, right_start, right_stop;
    volatile uint32_t left_ccr_before_stop, right_ccr_before_stop;
    volatile uint32_t completed;
} W4MotorPulseDiagnostic;
extern W4MotorPulseDiagnostic w4_motor_pulse_diag;
int W4MotorPulse_Init(void);
void W4MotorPulse_Poll(void);
void W4MotorPulse_TimeoutISR(void);
#endif
