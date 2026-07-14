#pragma once
#include <stdint.h>

void buzzer_init(void);
// Starts a tone and returns immediately. The buzzer is turned off after duration_ms.
void buzzer_tone(uint32_t freq_hz, uint32_t duration_ms);
void buzzer_off(void);
