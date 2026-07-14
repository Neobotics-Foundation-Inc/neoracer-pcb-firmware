#include "buzzer.h"
#include "config.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_timer.h"

// Buzzer uses LEDC_TIMER_1, LEDC_CH_BUZZER
static esp_timer_handle_t s_buzzer_timer = nullptr;

static void buzzer_set_off(void) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LEDC_CH_BUZZER, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LEDC_CH_BUZZER);
}

static void buzzer_timer_cb(void *) {
    buzzer_set_off();
}

static void buzzer_timer_init_once(void) {
    if (s_buzzer_timer != nullptr) {
        return;
    }

    esp_timer_create_args_t args = {};
    args.callback = buzzer_timer_cb;
    args.name = "buzzer";
    esp_timer_create(&args, &s_buzzer_timer);
}

void buzzer_init(void) {
    ledc_timer_config_t t = {};
    t.speed_mode      = LEDC_LOW_SPEED_MODE;
    t.duty_resolution = LEDC_TIMER_10_BIT;
    t.timer_num       = LEDC_TIMER_1;
    t.freq_hz         = 1000;
    t.clk_cfg         = LEDC_AUTO_CLK;
    ledc_timer_config(&t);

    ledc_channel_config_t ch = {};
    ch.gpio_num   = BUZZER_PIN;
    ch.speed_mode = LEDC_LOW_SPEED_MODE;
    ch.channel    = (ledc_channel_t)LEDC_CH_BUZZER;
    ch.timer_sel  = LEDC_TIMER_1;
    ch.duty       = 0;
    ch.hpoint     = 0;
    ledc_channel_config(&ch);

    buzzer_timer_init_once();
}

void buzzer_tone(uint32_t freq_hz, uint32_t duration_ms) {
    if (freq_hz == 0 || duration_ms == 0) {
        buzzer_off();
        return;
    }

    buzzer_timer_init_once();
    if (s_buzzer_timer == nullptr) {
        return;
    }
    esp_timer_stop(s_buzzer_timer);

    ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_1, freq_hz);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LEDC_CH_BUZZER, 512); // 50% duty
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LEDC_CH_BUZZER);

    if (esp_timer_start_once(s_buzzer_timer, (uint64_t)duration_ms * 1000) != ESP_OK) {
        buzzer_set_off();
    }
}

void buzzer_off(void) {
    if (s_buzzer_timer != nullptr) {
        esp_timer_stop(s_buzzer_timer);
    }
    buzzer_set_off();
}
