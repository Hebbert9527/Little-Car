/* Explicit board integration port. Compile this OR tests/mock_port.c. */
#include "w4_nav.h"
#include "main.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_uart.h"
#include <math.h>
extern TIM_HandleTypeDef htim4,htim12;
extern UART_HandleTypeDef huart3; /* member 1 provides week3_board */
static uint8_t rx_byte;
/* Reviewed single-encoder scheme (W4_SINGLE_ENCODER): RIGHT commanded drive sign,
   used by w4_pose_single.c to give the unsigned PA7 encoder a direction.
   0 when the right wheel is unpowered; pose keeps the last known direction
   for a bounded coasting interval and rejects ambiguous movement. */
int w4_single_cmd_dir;
uint32_t W4Port_Lock(void) { uint32_t p=__get_PRIMASK(); __disable_irq(); __DMB(); return p; }
void W4Port_Unlock(uint32_t p) { __DMB(); __set_PRIMASK(p); }
uint32_t W4Port_Millis(void) { return HAL_GetTick(); }
uint32_t W4Port_Micros(void) {
    /* Extend DWT before dividing: raw CYCCNT wraps every ~23.9 s at 180 MHz.
       Call at least once per wrap; the control task calls it every 10 ms. */
    static uint32_t previous; static uint64_t cycles;
    uint32_t k=W4Port_Lock(),now=DWT->CYCCNT;
    cycles+=(uint32_t)(now-previous); previous=now; uint64_t copy=cycles;
    W4Port_Unlock(k); return (uint32_t)(copy/(SystemCoreClock/1000000U));
}
void W4Board_EnableTiming(void) {
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT=0; DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
}
#ifdef W4_DUAL_ENCODER
static bool encoder_running(const TIM_TypeDef *timer) {
    const uint32_t channels = TIM_CCER_CC1E | TIM_CCER_CC2E;
    return (timer->CR1 & TIM_CR1_CEN) && (timer->SMCR & TIM_SMCR_SMS) == TIM_ENCODERMODE_TI12 &&
           (timer->CCER & channels) == channels && timer->ARR == 65535u;
}
#endif
bool W4Port_EncodersReady(void) {
#ifdef W4_ENCODER_AB_DIAG
    /* Diagnostic counters are not connected to navigation or calibrated yet. */
    return false;
#elif defined(W4_DUAL_ENCODER)
    return encoder_running(TIM3) && encoder_running(TIM8);
#elif defined(W4_SINGLE_ENCODER)
    /* Reviewed single-encoder dead reckoning: PA7 is ONE unsigned right-wheel
       channel; heading comes from the IMU, not wheel differential. This is NOT
       dual quadrature odometry. Ready only after the single channel is
       calibrated (w4_config.encoders_verified). See w4_pose_single.c. */
    return w4_config.encoders_verified;
#else
    /* Replace ONLY after wiring two independent inputs with measured direction.
       Existing PA7 is a single unsigned right encoder, so it is insufficient. */
    return false;
#endif
}
#ifdef W4_DUAL_ENCODER
bool W4Port_ReadEncoderCounts(uint16_t *left, uint16_t *right) {
    if (!W4Port_EncodersReady()) return false;
    *left = (uint16_t)TIM8->CNT;
    *right = (uint16_t)TIM3->CNT;
    return true;
}
#endif
static uint32_t ticks(float v,uint32_t top) {
    if(!isfinite(v)) return 0;
    return (uint32_t)(fminf(fabsf(v),1.0f)*(float)(top+1));
}
void W4Port_WritePwm(float left,float right) {
#ifdef W4_ENCODER_AB_DIAG
    /* Independent hard stop even if another readiness check is changed. */
    left=0; right=0;
#endif
    uint32_t l=ticks(left,__HAL_TIM_GET_AUTORELOAD(&htim12));
    uint32_t r=ticks(right,__HAL_TIM_GET_AUTORELOAD(&htim4));
    /* Original mapping: right positive CH1, left positive CH2 (inverted). */
    TIM4->CR1|=TIM_CR1_UDIS; TIM12->CR1|=TIM_CR1_UDIS;
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,right>0?r:0);
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,right<0?r:0);
    __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,left<0?l:0);
    __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,left>0?l:0);
    TIM4->CR1&=~TIM_CR1_UDIS; TIM12->CR1&=~TIM_CR1_UDIS;
    if(left==0&&right==0) { TIM4->EGR=TIM_EGR_UG; TIM12->EGR=TIM_EGR_UG; }
#ifdef W4_SINGLE_ENCODER
    w4_single_cmd_dir = right>0?1:right<0?-1:0;
#endif
}
bool W4Port_RxStart(void) { return HAL_UART_Receive_IT(&huart3,&rx_byte,1)==HAL_OK; }
bool W4Port_RxRecover(void) {
    /* Task context. Never HAL_UART_Abort: that also stops telemetry TX DMA. */
    if(HAL_UART_AbortReceive(&huart3)!=HAL_OK) return false;
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    return W4Port_RxStart();
}
/* Call from member 1's existing global callbacks; do not define duplicate HAL callbacks. */
void W4Bt_OnRxComplete(UART_HandleTypeDef *h) {
    if(h!=&huart3) return;
    W4Bt_OnByteISR(rx_byte);
    if(HAL_UART_Receive_IT(&huart3,&rx_byte,1)!=HAL_OK) W4Bt_OnErrorISR();
}
void W4Bt_OnUartError(UART_HandleTypeDef *h) {
    if(h==&huart3) W4Bt_OnErrorISR();
}
