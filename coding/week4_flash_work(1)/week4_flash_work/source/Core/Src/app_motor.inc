static void Motor_WritePWM(uint32_t ch1, uint32_t ch2,
                           uint32_t left1, uint32_t left2, uint32_t force_stop)
{
    TIM4->CR1 |= TIM_CR1_UDIS;
    TIM12->CR1 |= TIM_CR1_UDIS;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ch1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, ch2);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, left1);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, left2);
    TIM4->CR1 &= ~TIM_CR1_UDIS;
    TIM12->CR1 &= ~TIM_CR1_UDIS;
    if (force_stop) {
        TIM4->EGR = TIM_EGR_UG;
        TIM12->EGR = TIM_EGR_UG;
    }
    pwm_ch1_ticks = ch1;
    pwm_ch2_ticks = ch2;
    left_pwm_ch1_ticks = left1;
    left_pwm_ch2_ticks = left2;
}

static void Motor_Process(void)
{
    static int32_t pending_right = 0, pending_left = 0;
    static uint32_t stopped_ms = 0;
    uint32_t now = HAL_GetTick();
    int32_t requested = direction_request;
    uint32_t duty = pwm_test_mode ? pwm_test_duty_percent : motor_duty_percent;
    if (duty > 100U) duty = 100U;
    if (!motor_enable || (requested != 1 && requested != -1) || duty == 0U
        || (!right_motor_enable && !left_motor_enable)
        || (!key1_level && !key2_level)) {
        if (pwm_ch1_ticks || pwm_ch2_ticks || left_pwm_ch1_ticks || left_pwm_ch2_ticks)
            Motor_WritePWM(0,0,0,0,1);
        motor_direction_applied = 0;
        left_motor_direction_applied = 0;
        pending_right = pending_left = 0;
        stopped_ms = now;
        return;
    }
    int32_t right_direction = right_motor_enable ? requested : 0;
    int32_t left_direction = left_motor_enable ? (left_motor_invert ? -requested : requested) : 0;
    if (right_direction != motor_direction_applied || left_direction != left_motor_direction_applied) {
        if (pending_right != right_direction || pending_left != left_direction) {
            Motor_WritePWM(0,0,0,0,1);
            motor_direction_applied = 0;
            left_motor_direction_applied = 0;
            pending_right = right_direction;
            pending_left = left_direction;
            stopped_ms = now;
        }
         
        if ((uint32_t)(now-stopped_ms) < 50U) return;
        motor_direction_applied = right_direction;
        left_motor_direction_applied = left_direction;
    }
    uint32_t ticks = (4500U*duty + 50U)/100U;
    uint32_t ch1 = right_direction == 1 ? ticks : 0U;
    uint32_t ch2 = right_direction == -1 ? ticks : 0U;
    uint32_t left1 = left_direction == 1 ? ticks : 0U;
    uint32_t left2 = left_direction == -1 ? ticks : 0U;
    if (ch1 != pwm_ch1_ticks || ch2 != pwm_ch2_ticks
        || left1 != left_pwm_ch1_ticks || left2 != left_pwm_ch2_ticks)
        Motor_WritePWM(ch1,ch2,left1,left2,0);
}
