#pragma once
#include <stdint.h>

void servo_init(void);
void servo_set_pulse(int channel, int pulse_us);
int servo_duty_from_pulse(int pulse_us);
