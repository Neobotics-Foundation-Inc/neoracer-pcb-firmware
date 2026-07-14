#include "serial_cmd.h"
#include "app_state.h"
#include "config.h"
#include "fw_update.h"
#include "host_link.h"
#include "nvs_params.h"
#include "pid.h"
#include "imu.h"
#include "imu_heater.h"
#include "mag.h"
#include "serial_frames.h"
#include "serial_port.h"
#include "serial_tx.h"
#include "safety_event.h"
#include "steering_utils.h"
#include "version.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

extern pid_ctrl_t g_pid;
extern portMUX_TYPE g_state_mux;
extern QMC6309 s_mag;
extern void odom_reset_pose(void);

void serial_cmd_init(void) {
    ESP_ERROR_CHECK(serial_port_init());
    serial_tx_init();
}

static int steering_deg_to_pulse(float steering_deg) {
    if (fabsf(steering_deg) < 0.001f) {
        return STEERING_CENTER;
    }

    steering_deg = CONSTRAIN(steering_deg, -STEERING_MAX_ANGLE_DEG, STEERING_MAX_ANGLE_DEG);
    float pulse_f = MAP_FLOAT(steering_deg,
                              -STEERING_MAX_ANGLE_DEG, STEERING_MAX_ANGLE_DEG,
                              (float)STEERING_MIN, (float)STEERING_MAX);
    int pulse = (int)lroundf(pulse_f);

    if (pulse < STEERING_MIN) pulse = STEERING_MIN;
    if (pulse > STEERING_MAX) pulse = STEERING_MAX;
    return pulse;
}

void serial_cmd_process(const char *line) {
    if (!line || line[0] == '\0') return;

    if (fw_update_handle_line(line)) {
        return;
    }

    char cmd = line[0];

    switch (cmd) {
        case 'v': {
            if (fw_update_active()) {
                serial_tx_printf("ERROR: firmware update active\n");
                break;
            }

            taskENTER_CRITICAL(&g_state_mux);
            bool remote_active = g_state.remote_active;
            taskEXIT_CRITICAL(&g_state_mux);

            if (remote_active) {
                break;
            }

            float vx = 0.0f, steering = 0.0f;

            if (sscanf(line + 1, " %f %f", &vx, &steering) == 2) {
                if (vx > MAX_SPEED_FORWARD) vx = MAX_SPEED_FORWARD;
                if (vx < MAX_SPEED_REVERSE) vx = MAX_SPEED_REVERSE;
                if (fabsf(vx) < SPEED_STOP_THRESHOLD) vx = 0.0f;

                int steer_pulse = steering_deg_to_pulse(steering);
                unsigned long now_ms =
                    (unsigned long)(esp_timer_get_time() / 1000);

                taskENTER_CRITICAL(&g_state_mux);
                g_state.target_speed = vx;
                g_state.steering_pulse = steering_trimmed_pulse(steer_pulse, g_state.steering_trim_deg);
                g_state.serial_control_active = true;
                g_state.last_serial_cmd_ms = now_ms;
                taskEXIT_CRITICAL(&g_state_mux);
            }
            break;
        }

        case 'i':
            serial_frame_send_imu();
            break;

        case 'b':
            serial_frame_send_battery();
            break;

        case 'd':
            if (strcmp(line, "diag") == 0) {
                safety_diag_print_summary();
            } else {
                serial_tx_printf("ERROR: unknown command. Type 'help'\n");
            }
            break;

        case 'o':
            if (strncmp(line, "odom ", 5) == 0) {
                if (strncmp(line, "odom reset", 10) == 0) {
                    odom_reset_pose();
                    serial_tx_printf("OK: odom reset\n");
                } else if (strncmp(line, "odom get", 8) == 0) {
                    float pos[3], vel[3], yaw, scale;
                    taskENTER_CRITICAL(&g_state_mux);
                    memcpy(pos, g_state.odom_pos, sizeof(pos));
                    memcpy(vel, g_state.odom_vel, sizeof(vel));
                    yaw = g_state.odom_yaw;
                    scale = g_state.odom_scale;
                    taskEXIT_CRITICAL(&g_state_mux);
                    serial_tx_printf(
                        "ODOM: x=%.3f y=%.3f z=%.3f vx=%.3f vy=%.3f wz=%.3f yaw=%.4f scale=%.4f\n",
                        pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], yaw, scale);
                } else if (strncmp(line, "odom scale", 10) == 0) {
                    if (strncmp(line + 10, " set ", 5) == 0) {
                        float scale = ODOM_SCALE_DEFAULT;
                        if (sscanf(line + 15, "%f", &scale) == 1) {
                            scale = CONSTRAIN(scale, ODOM_SCALE_MIN, ODOM_SCALE_MAX);
                            float trim_deg = STEERING_TRIM_DEFAULT_DEG;
                            taskENTER_CRITICAL(&g_state_mux);
                            g_state.odom_scale = scale;
                            trim_deg = g_state.steering_trim_deg;
                            taskEXIT_CRITICAL(&g_state_mux);
                            if (nvs_chassis_save_async(scale, trim_deg)) {
                                serial_tx_printf("OK: odom scale=%.4f save queued\n", scale);
                            } else {
                                serial_tx_printf("ERROR: odom scale=%.4f active, NVS save queue full\n", scale);
                            }
                        } else {
                            serial_tx_printf("ERROR: odom scale set needs value\n");
                        }
                    } else if (strncmp(line + 10, " reset", 6) == 0) {
                        float trim_deg = STEERING_TRIM_DEFAULT_DEG;
                        taskENTER_CRITICAL(&g_state_mux);
                        g_state.odom_scale = ODOM_SCALE_DEFAULT;
                        trim_deg = g_state.steering_trim_deg;
                        taskEXIT_CRITICAL(&g_state_mux);
                        if (nvs_chassis_save_async(ODOM_SCALE_DEFAULT, trim_deg)) {
                            serial_tx_printf("OK: odom scale reset to %.4f, save queued\n", ODOM_SCALE_DEFAULT);
                        } else {
                            serial_tx_printf("ERROR: odom scale reset active, NVS save queue full\n");
                        }
                    } else if (strncmp(line + 10, " get", 4) == 0) {
                        taskENTER_CRITICAL(&g_state_mux);
                        float scale = g_state.odom_scale;
                        taskEXIT_CRITICAL(&g_state_mux);
                        serial_tx_printf("ODOM_SCALE: %.4f range=%.2f..%.2f\n",
                                         scale, ODOM_SCALE_MIN, ODOM_SCALE_MAX);
                    } else {
                        serial_tx_printf("ERROR: use odom get|reset|scale get|scale set <value>|scale reset\n");
                    }
                } else {
                    serial_tx_printf("ERROR: use odom get|reset|scale get|scale set <value>|scale reset\n");
                }
            } else {
                serial_frame_send_odom();
            }
            break;

        case 'm':
            if (strncmp(line, "mc ", 3) == 0) {
                const char *sub = line + 3;

                if (strncmp(sub, "cal", 3) == 0) {
                    uint32_t dur_ms = 30000;
                    sscanf(sub + 3, " %lu", &dur_ms);
                    if (dur_ms < 5000)   dur_ms = 5000;
                    if (dur_ms > 120000) dur_ms = 120000;

                    taskENTER_CRITICAL(&g_state_mux);
                    bool already = g_state.mag_cal_running;
                    taskEXIT_CRITICAL(&g_state_mux);

                    if (already) {
                        serial_tx_printf("WARN: mag cal already running\n");
                        break;
                    }

                    uint32_t *dur_arg = (uint32_t *)malloc(sizeof(uint32_t));
                    if (!dur_arg) {
                        serial_tx_printf("ERROR: malloc failed\n");
                        break;
                    }
                    *dur_arg = dur_ms;

                    taskENTER_CRITICAL(&g_state_mux);
                    g_state.mag_cal_running = true;
                    taskEXIT_CRITICAL(&g_state_mux);

                    xTaskCreate([](void *arg) {
                        uint32_t ms = *(uint32_t *)arg;
                        free(arg);

                        serial_tx_printf(
                            "INFO: mag cal started (%lus) — rotate 360 slowly\n",
                            (unsigned long)(ms / 1000));

                        s_mag.calibrate(ms);

                        float hard[3], soft[9];
                        s_mag.getCalibration(hard, soft);
                        bool saved = nvs_mag_save(hard, soft);

                        serial_tx_printf(
                            "%s: mag cal done"
                            " hi=[%.4f %.4f %.4f] Gauss"
                            " si_scale=[%.4f %.4f %.4f]%s\n",
                            saved ? "OK" : "ERROR",
                            hard[0], hard[1], hard[2],
                            soft[0], soft[4], soft[8],
                            saved ? " saved" : " NVS save failed");

                        taskENTER_CRITICAL(&g_state_mux);
                        g_state.mag_cal_running = false;
                        g_state.mag_calibrated = saved;
                        taskEXIT_CRITICAL(&g_state_mux);

                        vTaskDelete(NULL);
                    }, "mag_cal", 3072, dur_arg, 2, NULL);

                } else if (strncmp(sub, "set ", 4) == 0) {
                    float hard[3], soft[9];
                    int n = sscanf(sub + 4,
                        "%f %f %f %f %f %f %f %f %f %f %f %f",
                        &hard[0], &hard[1], &hard[2],
                        &soft[0], &soft[1], &soft[2],
                        &soft[3], &soft[4], &soft[5],
                        &soft[6], &soft[7], &soft[8]);
                    if (n == 12) {
                        // Input is in Tesla (ROS standard); convert hard-iron to Gauss.
                        // soft-iron matrix is dimensionless, no conversion needed.
                        hard[0] *= 1e4f;
                        hard[1] *= 1e4f;
                        hard[2] *= 1e4f;
                        s_mag.setCalibration(hard, soft);
                        bool queued = nvs_mag_save_async(hard, soft);
                        taskENTER_CRITICAL(&g_state_mux);
                        g_state.mag_calibrated = queued;
                        taskEXIT_CRITICAL(&g_state_mux);
                        if (queued) {
                            serial_tx_printf(
                                "OK: mag calib set, save queued"
                                " hi=[%.4f %.4f %.4f] Gauss\n",
                                hard[0], hard[1], hard[2]);
                        } else {
                            serial_tx_printf(
                                "ERROR: mag calib set active, NVS save queue full"
                                " hi=[%.4f %.4f %.4f] Gauss\n",
                                hard[0], hard[1], hard[2]);
                        }
                    } else {
                        serial_tx_printf(
                            "ERROR: mc set needs 12 floats"
                            " (hx hy hz s00..s22)\n");
                    }

                } else if (strncmp(sub, "get", 3) == 0) {
                    float hard[3], soft[9];
                    s_mag.getCalibration(hard, soft);
                    serial_tx_printf(
                        "MC: %.6f %.6f %.6f"
                        " %.6f %.6f %.6f"
                        " %.6f %.6f %.6f"
                        " %.6f %.6f %.6f\n",
                        hard[0], hard[1], hard[2],
                        soft[0], soft[1], soft[2],
                        soft[3], soft[4], soft[5],
                        soft[6], soft[7], soft[8]);

                } else if (strncmp(sub, "reset", 5) == 0) {
                    static const float zero[3] = {0.0f, 0.0f, 0.0f};
                    static const float ident[9] = {
                        1.0f, 0.0f, 0.0f,
                        0.0f, 1.0f, 0.0f,
                        0.0f, 0.0f, 1.0f,
                    };
                    s_mag.setCalibration(zero, ident);
                    bool queued = nvs_mag_save_async(zero, ident);
                    taskENTER_CRITICAL(&g_state_mux);
                    g_state.mag_calibrated = false;
                    taskEXIT_CRITICAL(&g_state_mux);
                    if (queued) {
                        serial_tx_printf("OK: mag calib reset to identity, save queued\n");
                    } else {
                        serial_tx_printf("ERROR: mag calib reset active, NVS save queue full\n");
                    }

                } else {
                    serial_tx_printf(
                        "ERROR: unknown mc subcommand."
                        " Use: set|get|reset\n");
                }
            } else {
                serial_frame_send_mag();
            }
            break;

        case 'k': {
            serial_tx_printf("ERROR: use pid get|set <kp> <ki> <kd>|reset\n");
            break;
        }

        case 'p':
            if (strncmp(line, "pid", 3) == 0) {
                if (strncmp(line, "pid set ", 8) == 0) {
                    float kp = 0.0f, ki = 0.0f, kd = 0.0f;
                    if (sscanf(line + 8, "%f %f %f", &kp, &ki, &kd) == 3) {
                        taskENTER_CRITICAL(&g_state_mux);
                        g_state.kp = kp;
                        g_state.ki = ki;
                        g_state.kd = kd;
                        g_pid.kp = kp;
                        g_pid.ki = ki;
                        g_pid.kd = kd;
                        pid_reset(&g_pid);
                        taskEXIT_CRITICAL(&g_state_mux);
                        if (nvs_params_save_async(kp, ki, kd)) {
                            serial_tx_printf("OK: pid=%.2f %.2f %.2f save queued\n", kp, ki, kd);
                        } else {
                            serial_tx_printf("ERROR: pid active, NVS save queue full\n");
                        }
                    } else {
                        serial_tx_printf("ERROR: pid set needs <kp> <ki> <kd>\n");
                    }
                } else if (strncmp(line, "pid reset", 9) == 0) {
                    taskENTER_CRITICAL(&g_state_mux);
                    g_state.kp = PID_KP_DEFAULT;
                    g_state.ki = PID_KI_DEFAULT;
                    g_state.kd = PID_KD_DEFAULT;
                    g_pid.kp = PID_KP_DEFAULT;
                    g_pid.ki = PID_KI_DEFAULT;
                    g_pid.kd = PID_KD_DEFAULT;
                    pid_reset(&g_pid);
                    taskEXIT_CRITICAL(&g_state_mux);
                    if (nvs_params_save_async(PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT)) {
                        serial_tx_printf("OK: PID reset to defaults, save queued\n");
                    } else {
                        serial_tx_printf("ERROR: PID reset active, NVS save queue full\n");
                    }
                } else if (strncmp(line, "pid get", 7) == 0) {
                    float kp = 0.0f, ki = 0.0f, kd = 0.0f;
                    taskENTER_CRITICAL(&g_state_mux);
                    kp = g_state.kp;
                    ki = g_state.ki;
                    kd = g_state.kd;
                    taskEXIT_CRITICAL(&g_state_mux);
                    serial_tx_printf("PID: %.2f %.2f %.2f\n", kp, ki, kd);
                } else {
                    serial_tx_printf("ERROR: use pid get|set <kp> <ki> <kd>|reset\n");
                }
            }
            break;

        case 's':
            if (strncmp(line, "s", 1) == 0 && line[1] == '\0') {
                serial_frame_send_sync();
            } else if (strncmp(line, "stream ", 7) == 0) {
                if (strcmp(line + 7, "sync") == 0) {
                    taskENTER_CRITICAL(&g_state_mux);
                    g_state.stream_enabled = true;
                    g_state.stream_legacy = false;
                    taskEXIT_CRITICAL(&g_state_mux);
                    serial_tx_printf("OK: stream sync\n");
                } else if (strcmp(line + 7, "off") == 0) {
                    taskENTER_CRITICAL(&g_state_mux);
                    g_state.stream_enabled = false;
                    taskEXIT_CRITICAL(&g_state_mux);
                    serial_tx_printf("OK: stream off\n");
                } else if (strcmp(line + 7, "legacy") == 0) {
                    taskENTER_CRITICAL(&g_state_mux);
                    g_state.stream_enabled = true;
                    g_state.stream_legacy = true;
                    taskEXIT_CRITICAL(&g_state_mux);
                    serial_tx_printf("OK: stream legacy\n");
                } else {
                    serial_tx_printf("ERROR: use stream sync|legacy|off\n");
                }
            } else if (strncmp(line, "stream", 6) == 0) {
                serial_tx_printf("ERROR: use stream sync|legacy|off\n");
            } else if (strncmp(line, "sync", 4) == 0) {
                serial_tx_printf("ERROR: use s or stream sync|legacy|off\n");
            } else if (strncmp(line, "sn get", 6) == 0) {
                uint8_t mac[6] = {0};
                if (esp_efuse_mac_get_default(mac) == ESP_OK) {
                    serial_tx_printf("SN: %02X%02X%02X%02X%02X%02X\n",
                                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                } else {
                    serial_tx_printf("ERROR: sn unavailable\n");
                }
            } else if (strncmp(line, "status", 6) == 0) {
                float bx = 0.0f, by = 0.0f, bz = 0.0f;
                float ox = 0.0f, oy = 0.0f, oz = 0.0f;
                float filtered_speed = 0.0f;
                float target_speed = 0.0f;
                float voltage = 0.0f;
                float temp = 0.0f;
                float odom_scale = ODOM_SCALE_DEFAULT;
                bool remote_active = false;
                bool speed_full_mode = false;
                bool is_static = false;
                bool mag_available = false;
                float steering_trim_deg = STEERING_TRIM_DEFAULT_DEG;
                int speed_deadband_us = THROTTLE_FF_DEADBAND_DEFAULT_US;
                imu_get_bias(&bx, &by, &bz);
                imu_get_level_offset(&ox, &oy, &oz);

                taskENTER_CRITICAL(&g_state_mux);
                filtered_speed = g_state.filtered_speed;
                target_speed = g_state.target_speed;
                voltage = g_state.voltage;
                temp = g_state.temp;
                odom_scale = g_state.odom_scale;
                remote_active = g_state.remote_active;
                speed_full_mode = g_state.speed_full_mode;
                is_static = g_state.is_static;
                mag_available = g_state.mag_available;
                steering_trim_deg = g_state.steering_trim_deg;
                speed_deadband_us = g_state.speed_deadband_us;
                taskEXIT_CRITICAL(&g_state_mux);

                serial_tx_printf(
                    "Status: Speed=%.3fm/s, Target=%.3fm/s, Voltage=%.1fV, "
                    "Control=%s, SpeedMode=%s, Static=%s\n",
                    filtered_speed,
                    target_speed,
                    voltage,
                    remote_active ? "Remote" : "Serial",
                    speed_full_mode ? "Full" : STRINGIFY(RC_LOW_SPEED_PERCENT) "%",
                    is_static ? "Yes" : "No"
                );
                serial_tx_printf(
                    "IMU: BiasReady=%s, LevelCal=%s, "
                    "GyroBias=%.4f,%.4f,%.4f, LevelOffset=%.4f,%.4f,%.4f\n",
                    imu_bias_ready() ? "Yes" : "No",
                    level_cal_is_calibrated() ? "Yes" : "No",
                    bx, by, bz,
                    ox, oy, oz
                );
                serial_tx_printf(
                    "Heater: Temp=%.1fC, State=%s(%.0fC) %s/%s/%s\n",
                    temp,
                    imu_heater_is_on() ? "ON" : "OFF",
                    imu_heater_get_target(),
                    imu_heater_warm() ? "WARM" : "COLD",
                    imu_heater_stable() ? "STABLE" : "UNSTABLE",
                    imu_heater_ready() ? "READY" : "WARMING"
                );
                serial_tx_printf(
                    "Diag: Mag=%s, SerialTxDrops=%lu\n",
                    mag_available ? "OK" : "UNAVAILABLE",
                    (unsigned long)serial_tx_drop_count()
                );
                serial_tx_printf(
                    "Chassis: OdomScale=%.4f, SteeringTrim=%.2fdeg, SpeedDeadband=%dus\n",
                    odom_scale,
                    steering_trim_deg,
                    speed_deadband_us
                );
                serial_tx_printf(
                    "FW: Product=%s, Firmware=%s, Hardware=%s, Version=%s, Git=%s, Dirty=%s, Build=%s, ProjectVer=%s\n",
                    OSCORE_PRODUCT_NAME,
                    OSCORE_FIRMWARE_NAME,
                    OSCORE_HARDWARE_NAME,
                    OSCORE_RELEASE_DATE,
                    OSCORE_GIT_SHA,
                    OSCORE_DIRTY_FLAG,
                    OSCORE_BUILD_TIME,
                    OSCORE_PROJECT_VER
                );
                safety_event_print_summary();
            } else if (strncmp(line, "speed deadband set ", 19) == 0) {
                int deadband_us = THROTTLE_FF_DEADBAND_DEFAULT_US;
                if (sscanf(line + 19, "%d", &deadband_us) == 1) {
                    deadband_us = CONSTRAIN(deadband_us,
                                            THROTTLE_FF_DEADBAND_MIN_US,
                                            THROTTLE_FF_DEADBAND_MAX_US);
                    taskENTER_CRITICAL(&g_state_mux);
                    g_state.speed_deadband_us = deadband_us;
                    pid_reset(&g_pid);
                    taskEXIT_CRITICAL(&g_state_mux);
                    if (nvs_speed_deadband_save_async(deadband_us)) {
                        serial_tx_printf("OK: speed deadband=%dus save queued\n", deadband_us);
                    } else {
                        serial_tx_printf("ERROR: speed deadband=%dus active, NVS save queue full\n", deadband_us);
                    }
                } else {
                    serial_tx_printf("ERROR: speed deadband set needs value_us\n");
                }
            } else if (strncmp(line, "speed deadband reset", 20) == 0) {
                taskENTER_CRITICAL(&g_state_mux);
                g_state.speed_deadband_us = THROTTLE_FF_DEADBAND_DEFAULT_US;
                pid_reset(&g_pid);
                taskEXIT_CRITICAL(&g_state_mux);
                if (nvs_speed_deadband_save_async(THROTTLE_FF_DEADBAND_DEFAULT_US)) {
                    serial_tx_printf("OK: speed deadband reset to %dus, save queued\n", THROTTLE_FF_DEADBAND_DEFAULT_US);
                } else {
                    serial_tx_printf("ERROR: speed deadband reset active, NVS save queue full\n");
                }
            } else if (strncmp(line, "speed deadband get", 18) == 0) {
                int deadband_us = THROTTLE_FF_DEADBAND_DEFAULT_US;
                taskENTER_CRITICAL(&g_state_mux);
                deadband_us = g_state.speed_deadband_us;
                taskEXIT_CRITICAL(&g_state_mux);
                serial_tx_printf("SPEED_DEADBAND: %dus range=%d..%dus\n",
                                 deadband_us,
                                 THROTTLE_FF_DEADBAND_MIN_US,
                                 THROTTLE_FF_DEADBAND_MAX_US);
            } else if (strncmp(line, "speed", 5) == 0) {
                serial_tx_printf("ERROR: use speed deadband get|set <value_us>|reset\n");
            } else {
                serial_tx_printf("ERROR: unknown command. Type 'help'\n");
            }
            break;

        case 't':
            if (strncmp(line, "trim set ", 9) == 0) {
                float trim_deg = 0.0f;
                if (sscanf(line + 9, "%f", &trim_deg) == 1) {
                    trim_deg = CONSTRAIN(trim_deg, STEERING_TRIM_MIN_DEG, STEERING_TRIM_MAX_DEG);

                    float odom_scale = ODOM_SCALE_DEFAULT;
                    taskENTER_CRITICAL(&g_state_mux);
                    g_state.steering_trim_deg = trim_deg;
                    g_state.steering_pulse = steering_trimmed_pulse(STEERING_CENTER, trim_deg);
                    odom_scale = g_state.odom_scale;
                    taskEXIT_CRITICAL(&g_state_mux);

                    if (nvs_chassis_save_async(odom_scale, trim_deg)) {
                        serial_tx_printf("OK: steering trim=%.2fdeg save queued\n", trim_deg);
                    } else {
                        serial_tx_printf("ERROR: steering trim=%.2fdeg active, NVS save queue full\n", trim_deg);
                    }
                } else {
                    serial_tx_printf("ERROR: trim set needs degrees\n");
                }
            } else if (strncmp(line, "trim reset", 10) == 0) {
                float odom_scale = ODOM_SCALE_DEFAULT;
                taskENTER_CRITICAL(&g_state_mux);
                g_state.steering_trim_deg = STEERING_TRIM_DEFAULT_DEG;
                g_state.steering_pulse = steering_trimmed_pulse(STEERING_CENTER, STEERING_TRIM_DEFAULT_DEG);
                odom_scale = g_state.odom_scale;
                taskEXIT_CRITICAL(&g_state_mux);

                if (nvs_chassis_save_async(odom_scale, STEERING_TRIM_DEFAULT_DEG)) {
                    serial_tx_printf("OK: steering trim reset to %.2fdeg, save queued\n", STEERING_TRIM_DEFAULT_DEG);
                } else {
                    serial_tx_printf("ERROR: steering trim reset active, NVS save queue full\n");
                }
            } else if (strncmp(line, "trim get", 8) == 0) {
                taskENTER_CRITICAL(&g_state_mux);
                float trim_deg = g_state.steering_trim_deg;
                int center = steering_trimmed_pulse(STEERING_CENTER, trim_deg);
                taskEXIT_CRITICAL(&g_state_mux);
                serial_tx_printf("TRIM: %.2fdeg center_pwm=%dus range=%.1f..%.1fdeg\n",
                                 trim_deg, center,
                                 STEERING_TRIM_MIN_DEG, STEERING_TRIM_MAX_DEG);
            } else {
                serial_tx_printf("ERROR: use trim get|set <deg>|reset\n");
            }
            break;

        case 'r':
            if (strncmp(line, "reset", 5) == 0) {
                serial_tx_printf("INFO: rebooting...\n");
                vTaskDelay(pdMS_TO_TICKS(50));
                esp_restart();
            }
            break;

        case 'l':
            if (strcmp(line, "link up ros") == 0) {
                host_link_up();
                serial_tx_printf("OK: link up ros\n");
            } else if (strcmp(line, "link ping ros") == 0) {
                host_link_ping();
            } else if (strcmp(line, "link down ros") == 0) {
                host_link_down();
                serial_tx_printf("OK: link down ros\n");
            } else if (strncmp(line, "link ", 5) == 0) {
                serial_tx_printf("ERROR: use link up|ping|down ros\n");
            } else if (strncmp(line, "level get", 9) == 0) {
                float ox, oy, oz;
                imu_get_level_offset(&ox, &oy, &oz);
                serial_tx_printf("LEVEL: offset=[%.4f %.4f %.4f]g\n", ox, oy, oz);

            } else if (strncmp(line, "level cal", 9) == 0) {
                level_cal_trigger();
                serial_tx_printf("OK: level cal started, keep still ~3s\n");

            } else if (strncmp(line, "level reset", 11) == 0) {
                level_cal_trigger();
                if (nvs_level_reset_async()) {
                    serial_tx_printf("OK: level cal reset to zero, auto-cal armed, save queued\n");
                } else {
                    serial_tx_printf("ERROR: level reset active, NVS save queue full\n");
                }
            }
            break;

        case 'h':
            if (strncmp(line, "help", 4) == 0) {
                serial_tx_printf("Commands:\n");
                serial_tx_printf("  v <vx> <steer>              set speed(m/s) and steering(deg)\n");
                serial_tx_printf("  i | b | o | m               query IMU, battery, odom, mag\n");
                serial_tx_printf("  s                           query synchronized odom/IMU snapshot frame\n");
                serial_tx_printf("  stream sync|legacy|off      sync=s/m/r/b, legacy=i/o/m/r/b\n");
                serial_tx_printf("  sn get                      query ESP32-S3 hardware serial hex\n");
                serial_tx_printf("  status                      system status and safety summary\n");
                serial_tx_printf("  odom get|reset              query/reset odometry state\n");
                serial_tx_printf("  odom scale get|set <v>|reset  odometry scale (0.50..1.50)\n");
                serial_tx_printf("  trim get|set <deg>|reset    steering trim (-5.0..5.0deg)\n");
                serial_tx_printf("  pid get|set <kp> <ki> <kd>|reset  speed PID gains\n");
                serial_tx_printf("  speed deadband get|set <us>|reset  throttle FF deadband\n");
                serial_tx_printf("  level cal|get|reset         accelerometer level calibration\n");
                serial_tx_printf("  mc cal [sec]                magnetometer calibration\n");
                serial_tx_printf("  mc set <12f>|get|reset      magnetometer calibration params\n");
                serial_tx_printf("  fw begin <size> <sha256>    begin serial OTA app update\n");
                serial_tx_printf("  fw data <seq> <hex>         send OTA data chunk\n");
                serial_tx_printf("  fw end|abort|status         finish/abort/query OTA update\n");
                serial_tx_printf("  fw version                  firmware version information\n");
                serial_tx_printf("  reset | help                reboot or show this message\n");
            }
            break;

        default:
            serial_tx_printf("ERROR: unknown command. Type 'help'\n");
            break;
    }
}
