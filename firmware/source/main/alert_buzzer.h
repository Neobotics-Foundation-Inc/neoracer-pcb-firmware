#pragma once
#include <stdint.h>

void alert_buzzer_init(void);

// Non-blocking: beep n times (on_ms high, off_ms low). Ignores call if already beeping.
void alert_buzzer_beep(int n_beeps, uint32_t on_ms, uint32_t off_ms);

void alert_buzzer_stop(void);
