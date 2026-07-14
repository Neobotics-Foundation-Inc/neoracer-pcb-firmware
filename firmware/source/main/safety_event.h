#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SAFETY_LEVEL_OK = 0,
    SAFETY_LEVEL_INFO,
    SAFETY_LEVEL_WARN,
    SAFETY_LEVEL_ERROR,
    SAFETY_LEVEL_FATAL,
} safety_level_t;

typedef enum {
    SAFETY_EVENT_LOW_VOLTAGE = 0,
    SAFETY_EVENT_RC_FAILSAFE,
    SAFETY_EVENT_RC_TAKEOVER,
    SAFETY_EVENT_SERIAL_TIMEOUT,
    SAFETY_EVENT_MAG_CALIBRATING,
    SAFETY_EVENT_IMU_HEATING,
    SAFETY_EVENT_IMU_NOT_READY,
    SAFETY_EVENT_IMU_BIAS_NOT_READY,
    SAFETY_EVENT_LEVEL_NOT_CALIBRATED,
    SAFETY_EVENT_MAG_UNAVAILABLE,
    SAFETY_EVENT_MAG_UNCALIBRATED,
    SAFETY_EVENT_OTA_ACTIVE,
    SAFETY_EVENT_HOST_LOST,
    SAFETY_EVENT_COUNT,
} safety_event_id_t;

typedef enum {
    DIAG_LEVEL_OK = 0,
    DIAG_LEVEL_WARN = 1,
    DIAG_LEVEL_ERROR = 2,
    DIAG_LEVEL_SERVICE = 3,
} diag_level_t;

typedef enum {
    DIAG_CODE_READY = 0,
    DIAG_CODE_INIT = 10,
    DIAG_CODE_HOST_LOST = 110,
    DIAG_CODE_LOW_BAT_ERROR = 210,
    DIAG_CODE_RC_FAILSAFE = 310,
    DIAG_CODE_IMU_NOT_READY = 400,
    DIAG_CODE_LEVEL_NOT_CALIBRATED = 420,
    DIAG_CODE_I2C_SENSOR_ERROR = 510,
    DIAG_CODE_OTA_ACTIVE = 700,
} diag_code_t;

typedef struct {
    uint32_t mask;
    safety_level_t level;
    bool allow_motion;
    const char *top_event;
} safety_snapshot_t;

typedef struct {
    diag_level_t level;
    diag_code_t code;
    const char *name;
    bool allow_motion;
    const char *control;
    float voltage;
    uint32_t uptime_s;
} diag_snapshot_t;

void safety_event_update(void);
safety_snapshot_t safety_event_snapshot(void);
diag_snapshot_t safety_diag_snapshot(void);
bool safety_event_active(safety_event_id_t id);
const char *safety_level_name(safety_level_t level);
const char *safety_event_name(safety_event_id_t id);
void safety_event_print_summary(void);
void safety_diag_print_summary(void);
