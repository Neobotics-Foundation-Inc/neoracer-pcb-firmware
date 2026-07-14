#include "host_link.h"

#include "app_state.h"
#include "config.h"
#include "fw_update.h"
#include "melody_player.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_seen = false;
static bool s_connected = false;
static bool s_timeout_announced = false;
static unsigned long s_last_ping_ms = 0;

extern app_state_t g_state;
extern portMUX_TYPE g_state_mux;

static unsigned long now_ms(void) {
    return (unsigned long)(esp_timer_get_time() / 1000);
}

static bool suppress_melody_now(void) {
    float voltage = 0.0f;
    taskENTER_CRITICAL(&g_state_mux);
    voltage = g_state.voltage;
    taskEXIT_CRITICAL(&g_state_mux);
    return (voltage > 1.0f && voltage < LOW_VOLTAGE_THRESHOLD) || fw_update_active();
}

void host_link_up(void) {
    taskENTER_CRITICAL(&s_mux);
    s_seen = true;
    s_connected = true;
    s_timeout_announced = false;
    s_last_ping_ms = now_ms();
    taskEXIT_CRITICAL(&s_mux);
    if (!suppress_melody_now()) {
        melody_play_host_connected();
    }
}

void host_link_ping(void) {
    taskENTER_CRITICAL(&s_mux);
    s_seen = true;
    s_connected = true;
    s_timeout_announced = false;
    s_last_ping_ms = now_ms();
    taskEXIT_CRITICAL(&s_mux);
}

void host_link_down(void) {
    taskENTER_CRITICAL(&s_mux);
    s_seen = true;
    s_connected = false;
    s_timeout_announced = true;
    taskEXIT_CRITICAL(&s_mux);
    if (!suppress_melody_now()) {
        melody_play_host_lost();
    }
}

bool host_link_lost(void) {
    bool lost = false;
    bool play = false;
    const unsigned long now = now_ms();

    taskENTER_CRITICAL(&s_mux);
    if (s_seen && s_connected && s_last_ping_ms != 0 && now - s_last_ping_ms > 5000UL) {
        s_connected = false;
        if (!s_timeout_announced) {
            s_timeout_announced = true;
            play = true;
        }
    }
    lost = s_seen && !s_connected;
    taskEXIT_CRITICAL(&s_mux);

    if (play) {
        if (!suppress_melody_now()) {
            melody_play_host_lost();
        }
    }
    return lost;
}
