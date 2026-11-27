volatile uint32_t adc_raw = 0;
volatile uint32_t adc_voltage_mv = 0;
volatile uint32_t adc_samples = 0;
volatile uint32_t adc_errors = 0;
volatile uint32_t motor_duty_percent = 0;  

 
volatile uint32_t key1_level = 1, key2_level = 1;
volatile uint32_t key1_count = 0, key2_count = 0;
volatile uint32_t key1_irq_count = 0, key2_irq_count = 0;
volatile uint32_t key1_edge_ms = 0, key2_edge_ms = 0;
volatile int32_t direction_request = 0;  
volatile uint32_t motor_enable = 1;  
volatile uint32_t right_motor_enable = 1, left_motor_enable = 1;

volatile uint32_t left_motor_invert = 1;
volatile uint32_t pwm_test_mode = 0;  
volatile uint32_t pwm_test_duty_percent = 25;
volatile uint32_t pwm_frequency_hz = 20000;
volatile uint32_t pwm_ch1_ticks = 0, pwm_ch2_ticks = 0;
volatile uint32_t left_pwm_ch1_ticks = 0, left_pwm_ch2_ticks = 0;
volatile int32_t motor_direction_applied = 0;
volatile int32_t left_motor_direction_applied = 0;
static uint32_t key_irq_seen[2] = {0,0}, key_armed[2] = {0,0};
static uint32_t key_candidate[2] = {1,1};
static uint32_t key_stable[2] = {1,1};
static uint32_t key_changed_ms[2] = {0,0};

#define ENCODER_EDGES_PER_OUTPUT_REV 330U
#define ENCODER_TIMEOUT_MS 5000U
volatile uint32_t encoder_valid = 0, encoder_timeout = 1;
volatile uint32_t encoder_period_us = 0, encoder_high_us = 0;
volatile float encoder_frequency_hz = 0.0f;
volatile float encoder_duty_percent = 0.0f, encoder_rpm = 0.0f;
volatile uint32_t encoder_edge_count = 0, encoder_capture_count = 0;
volatile uint32_t encoder_overcapture_count = 0, encoder_invalid_count = 0;
volatile uint32_t encoder_overflow_count = 0;

 
static uint64_t encoder_epoch_us = 0;
static uint64_t encoder_last_rise_us = 0, encoder_last_fall_us = 0;
static uint32_t encoder_have_rise = 0, encoder_have_fall = 0;

static volatile uint64_t capture_period_us = 0, capture_high_us = 0;
static volatile uint32_t capture_valid = 0, capture_last_rise_ms = 0;

volatile uint32_t swv_enabled = 1, swv_trace_active = 0;
volatile uint32_t swv_lines_sent = 0, swv_pending_bytes = 0;
volatile uint32_t swv_format_errors = 0;
static char swv_line[320];
static uint32_t swv_line_length = 0, swv_line_position = 0;
