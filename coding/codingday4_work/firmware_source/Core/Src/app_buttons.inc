static void Buttons_Process(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t levels[2] = {
        HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET,
        HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_SET
    };
    key1_level = levels[0];
    key2_level = levels[1];
    for (uint32_t i=0; i<2; i++) {
        uint32_t irq_count = i == 0 ? key1_irq_count : key2_irq_count;
        if (irq_count != key_irq_seen[i]) {
            key_irq_seen[i] = irq_count;
            key_armed[i] = 1;
        }
        if (levels[i] != key_candidate[i]) {
            key_candidate[i] = levels[i];
            key_changed_ms[i] = now;
        }
        uint32_t last_edge = i == 0 ? key1_edge_ms : key2_edge_ms;
        if (key_armed[i] && key_stable[i] != key_candidate[i]
            && (uint32_t)(now-key_changed_ms[i]) >= 20U
            && (uint32_t)(now-last_edge) >= 20U) {
            key_armed[i] = 0;
            key_stable[i] = key_candidate[i];
            if (key_stable[i] == 0U) {
                if (i == 0) { key1_count++; direction_request = 1; }
                else { key2_count++; direction_request = -1; }
            }
        }
    }
    if (key_stable[0] == 0U && key_stable[1] == 0U) direction_request = 0;
}
