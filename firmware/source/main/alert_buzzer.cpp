#include "alert_buzzer.h"
#include "config.h"

#include "driver/gpio.h"
#include "esp_timer.h"

static esp_timer_handle_t s_timer = nullptr;
static int      s_remaining = 0;
static uint32_t s_on_ms     = 0;
static uint32_t s_off_ms    = 0;
static bool     s_phase_on  = false;

static void set_pin(bool on) {
    gpio_set_level((gpio_num_t)ALERT_BUZZER_PIN, on ? 1 : 0);
}

static void timer_cb(void *) {
    if (s_phase_on) {
        set_pin(false);
        s_phase_on = false;
        s_remaining--;
        if (s_remaining > 0) {
            esp_timer_start_once(s_timer, (uint64_t)s_off_ms * 1000);
        }
    } else {
        set_pin(true);
        s_phase_on = true;
        esp_timer_start_once(s_timer, (uint64_t)s_on_ms * 1000);
    }
}

void alert_buzzer_init(void) {
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << ALERT_BUZZER_PIN;
    cfg.mode         = GPIO_MODE_OUTPUT;
    cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&cfg);
    set_pin(false);

    esp_timer_create_args_t args = {};
    args.callback = timer_cb;
    args.name     = "alert_buz";
    esp_timer_create(&args, &s_timer);
}

void alert_buzzer_beep(int n_beeps, uint32_t on_ms, uint32_t off_ms) {
    if (n_beeps <= 0 || s_timer == nullptr) return;
    if (s_remaining > 0) return;  // already beeping, don't interrupt

    s_on_ms    = on_ms;
    s_off_ms   = off_ms;
    s_remaining = n_beeps;
    s_phase_on  = true;
    set_pin(true);
    esp_timer_start_once(s_timer, (uint64_t)on_ms * 1000);
}

void alert_buzzer_stop(void) {
    if (s_timer) esp_timer_stop(s_timer);
    s_remaining = 0;
    s_phase_on  = false;
    set_pin(false);
}
