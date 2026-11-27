#ifndef W4_INTEGRATION_H
#define W4_INTEGRATION_H
/* Member 1 integration glue for the Week4 RTOS build (W4_ENABLE_RTOS).
 * Call W4Integration_Start() from main() AFTER HAL_Init/SystemClock_Config and
 * the MX_GPIO/ADC/TIM2/TIM3/TIM4/TIM12 init, INSTEAD of Week3_Init()+while(1).
 * It does not return: it starts the CMSIS-RTOS2 kernel. */
void W4Integration_Start(void);
/* Apply physical calibration to w4_config before tasks start. Keeps motion
   locked (verified=false) until W4_CALIBRATED is defined after measurement. */
void W4Calibration_Apply(void);
#endif