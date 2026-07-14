#include "status_indicator.h"

#include "alert_buzzer.h"
#include "app_state.h"
#include "buzzer.h"
#include "config.h"
#include "imu.h"
#include "imu_heater.h"
#include "led.h"
#include "safety_event.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern portMUX_TYPE g_state_mux;

typedef enum {
    IND_BOOT = 0,
    IND_LOW_VOLTAGE,
    IND_OTA,
    IND_RC_FAILSAFE,
    IND_HOST_LOST,
    IND_MAG_CAL,
    IND_HEATING,
    IND_WARM,
    IND_STABLE,
    IND_IMU_CAL,
    IND_READY,
} indicator_state_t;

typedef struct {
    safety_snapshot_t safety;
} indicator_snapshot_t;

static void read_snapshot(indicator_snapshot_t *s) {
    s->safety = safety_event_snapshot();
}

static bool snapshot_has_event(const indicator_snapshot_t *s, safety_event_id_t id) {
    return (s->safety.mask & (1UL << (uint32_t)id)) != 0;
}

static indicator_state_t choose_state(const indicator_snapshot_t *s) {
    if (snapshot_has_event(s, SAFETY_EVENT_LOW_VOLTAGE)) {
        return IND_LOW_VOLTAGE;
    }
    if (snapshot_has_event(s, SAFETY_EVENT_OTA_ACTIVE)) {
        return IND_OTA;
    }
    if (snapshot_has_event(s, SAFETY_EVENT_MAG_CALIBRATING)) {
        return IND_MAG_CAL;
    }
    if (!imu_heater_warm()) {
        return IND_HEATING;
    }
    if (!imu_heater_stable()) {
        return IND_WARM;
    }
    if (!imu_heater_ready()) {
        return IND_STABLE;
    }
    if (!imu_bias_ready() || !level_cal_is_calibrated()) {
        return IND_IMU_CAL;
    }
    if (snapshot_has_event(s, SAFETY_EVENT_RC_FAILSAFE)) {
        return IND_RC_FAILSAFE;
    }
    if (snapshot_has_event(s, SAFETY_EVENT_HOST_LOST)) {
        return IND_HOST_LOST;
    }
    return IND_READY;
}

static void tone_state_enter(indicator_state_t state) {
    switch (state) {
    case IND_LOW_VOLTAGE:
        alert_buzzer_beep(3, 180, 120);
        buzzer_tone(330, 250);
        break;
    case IND_RC_FAILSAFE:
        buzzer_tone(440, 180);
        break;
    case IND_OTA:
        buzzer_tone(740, 120);
        break;
    case IND_MAG_CAL:
        buzzer_tone(988, 120);
        break;
    case IND_WARM:
        buzzer_tone(880, 180);
        break;
    case IND_STABLE:
        buzzer_tone(1200, 150);
        break;
    case IND_IMU_CAL:
        buzzer_tone(1047, 120);
        break;
    case IND_READY:
        buzzer_tone(1568, 180);
        alert_buzzer_stop();
        break;
    default:
        break;
    }
}

static void led_for_state(indicator_state_t state, uint32_t tick) {
    bool on_250 = (tick % 2) == 0;
    bool on_500 = (tick % 4) < 2;
    bool blink_2x = (tick % 8) == 0 || (tick % 8) == 2;
    bool double_blue = (tick % 12) == 0 || (tick % 12) == 2;

    switch (state) {
    case IND_LOW_VOLTAGE:
        led_set_color(on_250 ? 255 : 0, 0, 0);
        break;
    case IND_RC_FAILSAFE:
        led_set_color(blink_2x ? 255 : 0, 0, blink_2x ? 255 : 0);
        break;
    case IND_HOST_LOST:
        led_set_color(0, 0, double_blue ? 255 : 0);
        break;
    case IND_OTA:
        led_set_color(on_500 ? 255 : 20, on_500 ? 100 : 8, 0);
        break;
    case IND_MAG_CAL:
        led_set_color(on_250 ? 255 : 0, on_250 ? 80 : 0, 0);
        break;
    case IND_HEATING:
        led_set_color(on_500 ? 253 : 0, on_500 ? 233 : 0, 0);
        break;
    case IND_WARM:
        led_set_color(253, 233, 16);
        break;
    case IND_STABLE:
        led_set_color(0, on_500 ? 255 : 40, 0);
        break;
    case IND_IMU_CAL:
        led_set_color(0, on_250 ? 180 : 30, on_250 ? 255 : 50);
        break;
    case IND_READY:
        led_set_color(0, 0, 255);
        break;
    case IND_BOOT:
    default:
        led_set_color(on_500 ? 255 : 0, on_500 ? 255 : 0, on_500 ? 255 : 0);
        break;
    }
}

static void task_indicator(void *) {
    indicator_state_t last_state = IND_BOOT;
    uint32_t tick = 0;
    int64_t last_low_voltage_alert_us = 0;

    tone_state_enter(IND_BOOT);

    while (1) {
        indicator_snapshot_t snap;
        read_snapshot(&snap);
        indicator_state_t state = choose_state(&snap);

        if (state != last_state) {
            tone_state_enter(state);
            last_state = state;
            tick = 0;
        }

        if (state == IND_LOW_VOLTAGE) {
            int64_t now = esp_timer_get_time();
            if (now - last_low_voltage_alert_us >=
                    (int64_t)LOW_VOLTAGE_ALERT_PERIOD_MS * 1000) {
                alert_buzzer_beep(3, 180, 120);
                last_low_voltage_alert_us = now;
            }
        }

        led_for_state(state, tick++);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void status_indicator_init(void) {
    xTaskCreate(task_indicator, "indicator", 2048, NULL, 2, NULL);
}
