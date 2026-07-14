#include "servo.h"
#include "config.h"
#include "driver/ledc.h"
#include <stdio.h>

void servo_init(void) {
    ledc_timer_config_t t = {};
    t.speed_mode      = LEDC_LOW_SPEED_MODE;
    t.duty_resolution = (ledc_timer_bit_t)PWM_RESOLUTION;
    t.timer_num       = LEDC_TIMER_0;
    t.freq_hz         = PWM_FREQUENCY;
    t.clk_cfg         = LEDC_AUTO_CLK;
    ledc_timer_config(&t);

    ledc_channel_config_t ch = {};
    ch.speed_mode = LEDC_LOW_SPEED_MODE;
    ch.timer_sel  = LEDC_TIMER_0;
    ch.duty       = 0;
    ch.hpoint     = 0;

    ch.gpio_num = THROTTLE_PIN;
    ch.channel  = (ledc_channel_t)LEDC_CH_THROTTLE;
    ledc_channel_config(&ch);

    ch.gpio_num = STEERING_PIN;
    ch.channel  = (ledc_channel_t)LEDC_CH_STEERING;
    ledc_channel_config(&ch);
}

int servo_duty_from_pulse(int pulse_us) {
    if (pulse_us < 1000) pulse_us = 1000;
    if (pulse_us > 2000) pulse_us = 2000;
    return (int)((float)pulse_us / 20000.0f * ((1 << PWM_RESOLUTION) - 1));
}

void servo_set_pulse(int channel, int pulse_us) {
    uint32_t duty = (uint32_t)servo_duty_from_pulse(pulse_us);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel);
}
