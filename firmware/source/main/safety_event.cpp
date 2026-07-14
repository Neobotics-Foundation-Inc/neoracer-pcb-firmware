#include "safety_event.h"

#include "app_state.h"
#include "config.h"
#include "fw_update.h"
#include "host_link.h"
#include "imu.h"
#include "imu_heater.h"
#include "serial_tx.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern app_state_t g_state;
extern portMUX_TYPE g_state_mux;

typedef struct {
    safety_event_id_t id;
    const char *name;
    safety_level_t level;
    bool allow_motion;
} safety_event_def_t;

static const safety_event_def_t k_events[] = {
    {SAFETY_EVENT_LOW_VOLTAGE,          "LOW_VOLTAGE",          SAFETY_LEVEL_ERROR, false},
    {SAFETY_EVENT_RC_FAILSAFE,          "RC_FAILSAFE",          SAFETY_LEVEL_WARN,  true},
    {SAFETY_EVENT_RC_TAKEOVER,          "RC_TAKEOVER",          SAFETY_LEVEL_INFO,  true},
    {SAFETY_EVENT_SERIAL_TIMEOUT,       "SERIAL_TIMEOUT",       SAFETY_LEVEL_WARN,  true},
    {SAFETY_EVENT_MAG_CALIBRATING,      "MAG_CALIBRATING",      SAFETY_LEVEL_INFO,  false},
    {SAFETY_EVENT_IMU_HEATING,          "IMU_HEATING",          SAFETY_LEVEL_INFO,  true},
    {SAFETY_EVENT_IMU_NOT_READY,        "IMU_NOT_READY",        SAFETY_LEVEL_WARN,  true},
    {SAFETY_EVENT_IMU_BIAS_NOT_READY,   "IMU_BIAS_NOT_READY",   SAFETY_LEVEL_WARN,  true},
    {SAFETY_EVENT_LEVEL_NOT_CALIBRATED, "LEVEL_NOT_CALIBRATED", SAFETY_LEVEL_WARN,  true},
    {SAFETY_EVENT_MAG_UNAVAILABLE,      "MAG_UNAVAILABLE",      SAFETY_LEVEL_WARN,  true},
    {SAFETY_EVENT_MAG_UNCALIBRATED,     "MAG_UNCALIBRATED",     SAFETY_LEVEL_WARN,  true},
    {SAFETY_EVENT_OTA_ACTIVE,           "OTA_ACTIVE",           SAFETY_LEVEL_INFO,  false},
    {SAFETY_EVENT_HOST_LOST,            "HOST_LOST",            SAFETY_LEVEL_WARN,  true},
};

static uint32_t s_mask = 0;
static safety_level_t s_level = SAFETY_LEVEL_OK;
static bool s_allow_motion = true;
static safety_event_id_t s_top_event = SAFETY_EVENT_COUNT;
static portMUX_TYPE s_safety_mux = portMUX_INITIALIZER_UNLOCKED;

static bool s_low_voltage_latched = false;
static int64_t s_low_since_us = 0;
static int64_t s_recover_since_us = 0;

static uint32_t bit_for(safety_event_id_t id) {
    return 1UL << (uint32_t)id;
}

static void set_event(uint32_t *mask, safety_event_id_t id, bool active) {
    if (active) {
        *mask |= bit_for(id);
    }
}

static bool update_low_voltage_latch(float voltage) {
    bool latched = false;
    taskENTER_CRITICAL(&s_safety_mux);

    if (voltage <= 1.0f) {
        s_low_since_us = 0;
        s_recover_since_us = 0;
        latched = s_low_voltage_latched;
        taskEXIT_CRITICAL(&s_safety_mux);
        return latched;
    }

    int64_t now = esp_timer_get_time();
    const int64_t confirm_us = (int64_t)LOW_VOLTAGE_CONFIRM_MS * 1000;
    const int64_t recover_us = (int64_t)LOW_VOLTAGE_RECOVER_MS * 1000;

    if (!s_low_voltage_latched) {
        s_recover_since_us = 0;
        if (voltage < LOW_VOLTAGE_THRESHOLD) {
            if (s_low_since_us == 0) {
                s_low_since_us = now;
            } else if (now - s_low_since_us >= confirm_us) {
                s_low_voltage_latched = true;
                s_low_since_us = 0;
            }
        } else {
            s_low_since_us = 0;
        }
        latched = s_low_voltage_latched;
        taskEXIT_CRITICAL(&s_safety_mux);
        return latched;
    }

    s_low_since_us = 0;
    if (voltage > LOW_VOLTAGE_RECOVER) {
        if (s_recover_since_us == 0) {
            s_recover_since_us = now;
        } else if (now - s_recover_since_us >= recover_us) {
            s_low_voltage_latched = false;
            s_recover_since_us = 0;
        }
    } else {
        s_recover_since_us = 0;
    }

    latched = s_low_voltage_latched;
    taskEXIT_CRITICAL(&s_safety_mux);
    return latched;
}

void safety_event_update(void) {
    bool failsafe = false;
    bool remote_active = false;
    bool serial_control_active = false;
    bool mag_available = false;
    bool mag_calibrated = false;
    bool mag_cal_running = false;
    float voltage = 0.0f;
    unsigned long last_serial_cmd_ms = 0;

    taskENTER_CRITICAL(&g_state_mux);
    failsafe = g_state.failsafe;
    remote_active = g_state.remote_active;
    serial_control_active = g_state.serial_control_active;
    mag_available = g_state.mag_available;
    mag_calibrated = g_state.mag_calibrated;
    mag_cal_running = g_state.mag_cal_running;
    voltage = g_state.voltage;
    last_serial_cmd_ms = g_state.last_serial_cmd_ms;
    taskEXIT_CRITICAL(&g_state_mux);

    unsigned long now_ms = (unsigned long)(esp_timer_get_time() / 1000);
    bool serial_timeout = serial_control_active && !remote_active &&
        last_serial_cmd_ms != 0 &&
        now_ms - last_serial_cmd_ms > (unsigned long)SERIAL_TIMEOUT;

    uint32_t mask = 0;
    set_event(&mask, SAFETY_EVENT_LOW_VOLTAGE, update_low_voltage_latch(voltage));
    set_event(&mask, SAFETY_EVENT_RC_FAILSAFE, failsafe);
    set_event(&mask, SAFETY_EVENT_RC_TAKEOVER, remote_active);
    set_event(&mask, SAFETY_EVENT_SERIAL_TIMEOUT, serial_timeout);
    set_event(&mask, SAFETY_EVENT_MAG_CALIBRATING, mag_cal_running);
    set_event(&mask, SAFETY_EVENT_IMU_HEATING, !imu_heater_warm());
    set_event(&mask, SAFETY_EVENT_IMU_NOT_READY, !imu_heater_ready());
    set_event(&mask, SAFETY_EVENT_IMU_BIAS_NOT_READY, !imu_bias_ready());
    set_event(&mask, SAFETY_EVENT_LEVEL_NOT_CALIBRATED, !level_cal_is_calibrated());
    set_event(&mask, SAFETY_EVENT_MAG_UNAVAILABLE, !mag_available);
    set_event(&mask, SAFETY_EVENT_MAG_UNCALIBRATED, mag_available && !mag_calibrated);
    set_event(&mask, SAFETY_EVENT_OTA_ACTIVE, fw_update_active());
    set_event(&mask, SAFETY_EVENT_HOST_LOST, host_link_lost());

    safety_level_t level = SAFETY_LEVEL_OK;
    bool allow_motion = true;
    safety_event_id_t top_event = SAFETY_EVENT_COUNT;

    for (int i = 0; i < (int)(sizeof(k_events) / sizeof(k_events[0])); i++) {
        const safety_event_def_t *ev = &k_events[i];
        if ((mask & bit_for(ev->id)) == 0) {
            continue;
        }
        if (ev->level > level) {
            level = ev->level;
            top_event = ev->id;
        }
        if (!ev->allow_motion) {
            allow_motion = false;
        }
    }

    taskENTER_CRITICAL(&s_safety_mux);
    s_mask = mask;
    s_level = level;
    s_allow_motion = allow_motion;
    s_top_event = top_event;
    taskEXIT_CRITICAL(&s_safety_mux);
}

safety_snapshot_t safety_event_snapshot(void) {
    safety_event_update();
    safety_snapshot_t snap = {};
    taskENTER_CRITICAL(&s_safety_mux);
    snap.mask = s_mask;
    snap.level = s_level;
    snap.allow_motion = s_allow_motion;
    safety_event_id_t top_event = s_top_event;
    taskEXIT_CRITICAL(&s_safety_mux);
    snap.top_event = top_event == SAFETY_EVENT_COUNT ? "NONE" : safety_event_name(top_event);
    return snap;
}

static diag_snapshot_t diag_from_snapshot(const safety_snapshot_t *snap) {
    diag_snapshot_t diag = {};
    diag.level = DIAG_LEVEL_OK;
    diag.code = DIAG_CODE_READY;
    diag.name = "READY";
    diag.allow_motion = snap->allow_motion;

    const uint32_t mask = snap->mask;
    const auto has = [mask](safety_event_id_t id) {
        return (mask & bit_for(id)) != 0;
    };

    if (has(SAFETY_EVENT_LOW_VOLTAGE)) {
        diag.level = DIAG_LEVEL_ERROR;
        diag.code = DIAG_CODE_LOW_BAT_ERROR;
        diag.name = "LOW_BAT_ERROR";
    } else if (has(SAFETY_EVENT_OTA_ACTIVE)) {
        diag.level = DIAG_LEVEL_SERVICE;
        diag.code = DIAG_CODE_OTA_ACTIVE;
        diag.name = "OTA_ACTIVE";
    } else if (has(SAFETY_EVENT_IMU_NOT_READY) || has(SAFETY_EVENT_IMU_BIAS_NOT_READY)) {
        diag.level = DIAG_LEVEL_WARN;
        diag.code = DIAG_CODE_IMU_NOT_READY;
        diag.name = "IMU_NOT_READY";
    } else if (has(SAFETY_EVENT_LEVEL_NOT_CALIBRATED)) {
        diag.level = DIAG_LEVEL_WARN;
        diag.code = DIAG_CODE_LEVEL_NOT_CALIBRATED;
        diag.name = "LEVEL_NOT_CALIBRATED";
    } else if (has(SAFETY_EVENT_RC_FAILSAFE)) {
        diag.level = DIAG_LEVEL_WARN;
        diag.code = DIAG_CODE_RC_FAILSAFE;
        diag.name = "RC_FAILSAFE";
    } else if (has(SAFETY_EVENT_HOST_LOST)) {
        diag.level = DIAG_LEVEL_WARN;
        diag.code = DIAG_CODE_HOST_LOST;
        diag.name = "HOST_LOST";
    } else if (has(SAFETY_EVENT_IMU_HEATING)) {
        diag.level = DIAG_LEVEL_WARN;
        diag.code = DIAG_CODE_INIT;
        diag.name = "INIT";
    }

    return diag;
}

diag_snapshot_t safety_diag_snapshot(void) {
    safety_snapshot_t snap = safety_event_snapshot();
    diag_snapshot_t diag = diag_from_snapshot(&snap);

    bool remote_active = false;
    float voltage = 0.0f;
    taskENTER_CRITICAL(&g_state_mux);
    remote_active = g_state.remote_active;
    voltage = g_state.voltage;
    taskEXIT_CRITICAL(&g_state_mux);

    diag.control = remote_active ? "Remote" : "Serial";
    diag.voltage = voltage;
    diag.uptime_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    return diag;
}

bool safety_event_active(safety_event_id_t id) {
    safety_event_update();
    taskENTER_CRITICAL(&s_safety_mux);
    bool active = (s_mask & bit_for(id)) != 0;
    taskEXIT_CRITICAL(&s_safety_mux);
    return active;
}

const char *safety_level_name(safety_level_t level) {
    switch (level) {
    case SAFETY_LEVEL_INFO:  return "INFO";
    case SAFETY_LEVEL_WARN:  return "WARN";
    case SAFETY_LEVEL_ERROR: return "ERROR";
    case SAFETY_LEVEL_FATAL: return "FATAL";
    case SAFETY_LEVEL_OK:
    default:
        return "OK";
    }
}

const char *safety_event_name(safety_event_id_t id) {
    for (int i = 0; i < (int)(sizeof(k_events) / sizeof(k_events[0])); i++) {
        if (k_events[i].id == id) {
            return k_events[i].name;
        }
    }
    return "UNKNOWN";
}

void safety_event_print_summary(void) {
    safety_snapshot_t snap = safety_event_snapshot();
    serial_tx_printf(
        "SAFETY: Level=%s, AllowMotion=%s, Top=%s, Mask=0x%08lx\n",
        safety_level_name(snap.level),
        snap.allow_motion ? "Yes" : "No",
        snap.top_event,
        (unsigned long)snap.mask
    );
}

void safety_diag_print_summary(void) {
    diag_snapshot_t diag = safety_diag_snapshot();
    serial_tx_printf(
        "DIAG: Level=%d, Code=%d, Name=%s, Motion=%s, Control=%s, V=%.1f, Uptime=%lu\n",
        (int)diag.level,
        (int)diag.code,
        diag.name,
        diag.allow_motion ? "Yes" : "No",
        diag.control,
        diag.voltage,
        (unsigned long)diag.uptime_s
    );
}
