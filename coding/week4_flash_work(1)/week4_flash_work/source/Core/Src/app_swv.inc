static void SWV_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF0_TRACE;
    HAL_GPIO_Init(GPIOB, &gpio);
     
    DBGMCU->CR = (DBGMCU->CR & ~DBGMCU_CR_TRACE_MODE) | DBGMCU_CR_TRACE_IOEN;
}

static void SWV_Process(void)
{
    static uint32_t last_line_ms = 0;
    uint32_t now = HAL_GetTick();
    swv_trace_active = ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0U)
        && ((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0U) && ((ITM->TER & 1U) != 0U);
    if (!swv_enabled || !swv_trace_active) {
        swv_line_length = swv_line_position = swv_pending_bytes = 0;
        return;
    }
    if (swv_line_position == swv_line_length) {
        if ((uint32_t)(now-last_line_ms) < 200U) return;
        last_line_ms = now;
        

        uint32_t frequency10 = (uint32_t)(encoder_frequency_hz*10.0f + 0.5f);
        uint32_t rpm10 = (uint32_t)(encoder_rpm*10.0f + 0.5f);
        uint32_t duty10 = (uint32_t)(encoder_duty_percent*10.0f + 0.5f);
        uint32_t applied_pwm = (100U*(pwm_ch1_ticks+pwm_ch2_ticks)+2250U)/4500U;
        uint32_t left_pwm = (100U*(left_pwm_ch1_ticks+left_pwm_ch2_ticks)+2250U)/4500U;
        int length = snprintf(swv_line, sizeof(swv_line),
            "ms=%lu ADC=%lu SET=%lu%% PWM=%lu%% LPWM=%lu%% DIR=%ld EN=%lu RE=%lu LE=%lu V=%lu TO=%lu "
            "P=%luus H=%luus F=%lu.%luHz RPM=%lu.%lu ED=%lu.%lu%% BAD=%lu OF=%lu\r\n",
            (unsigned long)now, (unsigned long)adc_raw,
            (unsigned long)motor_duty_percent, (unsigned long)applied_pwm,
            (unsigned long)left_pwm,
            (long)direction_request,
            (unsigned long)motor_enable, (unsigned long)right_motor_enable,
            (unsigned long)left_motor_enable, (unsigned long)encoder_valid,
            (unsigned long)encoder_timeout, (unsigned long)encoder_period_us,
            (unsigned long)encoder_high_us,
            (unsigned long)(frequency10/10U), (unsigned long)(frequency10%10U),
            (unsigned long)(rpm10/10U), (unsigned long)(rpm10%10U),
            (unsigned long)(duty10/10U), (unsigned long)(duty10%10U),
            (unsigned long)encoder_invalid_count, (unsigned long)encoder_overcapture_count);
        if (length < 0 || (uint32_t)length >= sizeof(swv_line)) {
            swv_format_errors++;
            swv_line_length = swv_line_position = swv_pending_bytes = 0;
            return;
        }
        swv_line_length = (uint32_t)length;
        swv_line_position = 0;
    }
    

    for (uint32_t budget = 0; budget < 16U && swv_line_position < swv_line_length; budget++) {
        if (ITM->PORT[0].u32 == 0U) break;
        ITM->PORT[0].u8 = (uint8_t)swv_line[swv_line_position++];
        if (swv_line_position == swv_line_length) swv_lines_sent++;
    }
    swv_pending_bytes = swv_line_length - swv_line_position;
}
