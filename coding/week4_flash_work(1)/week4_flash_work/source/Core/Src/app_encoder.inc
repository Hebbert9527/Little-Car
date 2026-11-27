static void Encoder_Rising(uint64_t timestamp)
{
    encoder_edge_count++;
    capture_last_rise_ms = HAL_GetTick();
    if (encoder_have_rise) {
        uint64_t period = timestamp - encoder_last_rise_us;
        if (encoder_have_fall && encoder_last_fall_us > encoder_last_rise_us
            && encoder_last_fall_us < timestamp && period < 5000000ULL) {
            capture_period_us = period;
            capture_high_us = encoder_last_fall_us - encoder_last_rise_us;
            capture_valid = 1;
            encoder_capture_count++;
        } else {
            capture_valid = 0;
            encoder_invalid_count++;
        }
    }
    encoder_last_rise_us = timestamp;
    encoder_have_rise = 1;
    encoder_have_fall = 0;
}

static void Encoder_Falling(uint64_t timestamp)
{
    if (!encoder_have_rise) return;  
    if (encoder_have_fall || timestamp <= encoder_last_rise_us) {
        encoder_have_rise = encoder_have_fall = 0;
        capture_valid = 0;
        encoder_invalid_count++;
        return;
    }
    encoder_last_fall_us = timestamp;
    encoder_have_fall = 1;
}

void Encoder_TIM3_IRQHandler(void)
{
    uint32_t status = TIM3->SR;
    uint32_t have_rising = status & TIM_SR_CC2IF;
    uint32_t have_falling = status & TIM_SR_CC1IF;
    

    uint32_t rising = have_rising ? TIM3->CCR2 : 0U;
    uint32_t falling = have_falling ? TIM3->CCR1 : 0U;
    uint32_t count = TIM3->CNT;
    uint32_t overcapture = (status | TIM3->SR) & (TIM_SR_CC1OF | TIM_SR_CC2OF);
    uint64_t base = encoder_epoch_us;
    uint64_t rising_time = base + rising;
    uint64_t falling_time = base + falling;
    uint32_t clear_flags = overcapture;

    if (status & TIM_SR_UIF) {
        

        if (have_rising && rising <= count) rising_time += 65536ULL;
        if (have_falling && falling <= count) falling_time += 65536ULL;
        encoder_epoch_us = base + 65536ULL;
        encoder_overflow_count++;
        clear_flags |= TIM_SR_UIF;
    }
    if (clear_flags) TIM3->SR = ~clear_flags;  
    if (overcapture) {
        if (overcapture & TIM_SR_CC1OF) encoder_overcapture_count++;
        if (overcapture & TIM_SR_CC2OF) encoder_overcapture_count++;
        encoder_have_rise = encoder_have_fall = 0;
        capture_valid = 0;
        return;  
    }
    if (have_rising && have_falling) {
        if (falling_time < rising_time) {
            Encoder_Falling(falling_time);
            Encoder_Rising(rising_time);
        } else {
            Encoder_Rising(rising_time);
            Encoder_Falling(falling_time);
        }
    } else if (have_rising) {
        Encoder_Rising(rising_time);
    } else if (have_falling) {
        Encoder_Falling(falling_time);
    }
}

static void Encoder_Process(void)
{
    static uint32_t last_publish_ms = 0;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now-last_publish_ms) < 20U) return;
    last_publish_ms = now;

    uint32_t saved_primask = __get_PRIMASK();
    __disable_irq();
    now = HAL_GetTick();  
    uint64_t period = capture_period_us, high = capture_high_us;
    uint32_t valid = capture_valid, last_rise_ms = capture_last_rise_ms;
    uint32_t edges = encoder_edge_count;
    __set_PRIMASK(saved_primask);

    encoder_timeout = !edges || (uint32_t)(now-last_rise_ms) >= ENCODER_TIMEOUT_MS;
    encoder_valid = valid && !encoder_timeout && period > 0ULL
        && period < 5000000ULL && high > 0ULL && high < period;
    if (encoder_valid) {
        encoder_period_us = (uint32_t)period;
        encoder_high_us = (uint32_t)high;
        encoder_frequency_hz = 1000000.0f / (float)period;
        encoder_duty_percent = 100.0f * (float)high / (float)period;
        encoder_rpm = encoder_frequency_hz * (60.0f / ENCODER_EDGES_PER_OUTPUT_REV);
    } else {
        encoder_period_us = encoder_high_us = 0;
        encoder_frequency_hz = encoder_duty_percent = encoder_rpm = 0.0f;
    }
}
