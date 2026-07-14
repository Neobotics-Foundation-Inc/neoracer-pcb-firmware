#include "app_state.h"
#include "config.h"
#include "imu.h"
#include "imu_heater.h"
#include "madgwick.h"
#include "mag.h"
#include "encoder.h"
#include "servo.h"
#include "buzzer.h"
#include "alert_buzzer.h"
#include "melody_player.h"
#include "led.h"
#include "sbus.h"
#include "pid.h"
#include "nvs_params.h"
#include "serial_cmd.h"
#include "serial_frames.h"
#include "serial_port.h"
#include "serial_tx.h"
#include "safety_event.h"
#include "status_indicator.h"
#include "fw_log.h"
#include "fw_update.h"
#include "steering_utils.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

app_state_t g_state;
portMUX_TYPE g_state_mux = portMUX_INITIALIZER_UNLOCKED;
pid_ctrl_t g_pid;

static Madgwick s_madgwick;
static float    s_yaw_drift_rate   = 0.0f;
static float    s_odom_yaw_zero    = 0.0f;
static float    s_mag_hdg_ema      = 0.0f;
static bool     s_mag_hdg_ema_init = false;
static bool     s_bias_calibrated  = false;
static bool     s_level_calibrated = false;
static float    s_level_sum[3]     = {0};
static int      s_level_count      = 0;
static int      s_level_skip       = 0;

void level_cal_trigger(void) {
    s_level_calibrated = false;
    s_level_sum[0] = s_level_sum[1] = s_level_sum[2] = 0.0f;
    s_level_count = 0;
    s_level_skip  = 0;
    imu_set_level_offset(0.0f, 0.0f, 0.0f);
}

bool level_cal_is_calibrated(void) {
    return s_level_calibrated;
}

void odom_reset_pose(void) {
    taskENTER_CRITICAL(&g_state_mux);
    g_state.odom_pos[0] = 0.0f;
    g_state.odom_pos[1] = 0.0f;
    g_state.odom_pos[2] = 0.0f;
    g_state.odom_vel[0] = 0.0f;
    g_state.odom_vel[1] = 0.0f;
    g_state.odom_vel[2] = 0.0f;
    s_odom_yaw_zero = g_state.imu_yaw_raw;
    g_state.odom_yaw = 0.0f;
    taskEXIT_CRITICAL(&g_state_mux);
}

QMC6309  s_mag;
static adc_oneshot_unit_handle_t s_adc;

static const int   ADC_PTS[]  = {1017,1135,1197,1257,1379,1450,1495,1555,1615};
static const float VOLT_PTS[] = {9.0f,10.0f,10.5f,11.0f,12.0f,12.6f,13.0f,13.5f,14.0f};

static float adc_to_voltage(int raw) {
    if (raw <= ADC_PTS[0]) return VOLT_PTS[0];
    if (raw >= ADC_PTS[NUM_ADC_POINTS - 1]) return VOLT_PTS[NUM_ADC_POINTS - 1];

    for (int i = 0; i < NUM_ADC_POINTS - 1; i++) {
        if (raw >= ADC_PTS[i] && raw < ADC_PTS[i + 1]) {
            float r = (float)(raw - ADC_PTS[i]) / (ADC_PTS[i + 1] - ADC_PTS[i]);
            return VOLT_PTS[i] + r * (VOLT_PTS[i + 1] - VOLT_PTS[i]);
        }
    }

    return 0.0f;
}

static inline int map_range(int x, int in_min, int in_max, int out_min, int out_max) {
    return (int)((float)(x - in_min) * (out_max - out_min) /
                 (in_max - in_min) + out_min);
}

static inline int iclamp(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline int apply_deadzone(int v, int center, int dz) {
    return (abs(v - center) < dz) ? center : v;
}

static inline float wrap_pi(float angle) {
    while (angle >  3.14159265f) angle -= 2.0f * 3.14159265f;
    while (angle < -3.14159265f) angle += 2.0f * 3.14159265f;
    return angle;
}

static inline void yaw_relative_quat(float yaw_zero, float qw, float qx, float qy, float qz,
                                     float out[4]) {
    float half = -0.5f * yaw_zero;
    float zw = cosf(half);
    float zz = sinf(half);

    out[0] = zw * qw - zz * qz;
    out[1] = zw * qx - zz * qy;
    out[2] = zw * qy + zz * qx;
    out[3] = zw * qz + zz * qw;

    float norm = sqrtf(out[0] * out[0] + out[1] * out[1] +
                       out[2] * out[2] + out[3] * out[3]);
    if (norm > 1e-6f) {
        float inv = 1.0f / norm;
        out[0] *= inv;
        out[1] *= inv;
        out[2] *= inv;
        out[3] *= inv;
    }
}

static int throttle_feedforward_neutral(float target_speed, int deadband_us) {
    if (fabsf(target_speed) < SPEED_STOP_THRESHOLD) {
        return THROTTLE_NEUTRAL;
    }

    deadband_us = CONSTRAIN(deadband_us, THROTTLE_FF_DEADBAND_MIN_US, THROTTLE_FF_DEADBAND_MAX_US);
    int ff = (int)lroundf(deadband_us +
                          fabsf(target_speed) * THROTTLE_FF_GAIN_US_PER_MPS);
    ff = CONSTRAIN(ff, deadband_us, THROTTLE_FF_MAX_US);
    return THROTTLE_NEUTRAL + (target_speed > 0.0f ? ff : -ff);
}

static void send_rc_frame(void) {
    int ch[10];

    taskENTER_CRITICAL(&g_state_mux);
    bool failsafe = g_state.failsafe;
    for (int i = 0; i < 10; i++) {
        ch[i] = failsafe ? -1 : g_state.rc_ch[i];
    }
    taskEXIT_CRITICAL(&g_state_mux);

    serial_tx_printf(
        "r %d %d %d %d %d %d %d %d %d %d\n",
        ch[0], ch[1], ch[2], ch[3], ch[4],
        ch[5], ch[6], ch[7], ch[8], ch[9]
    );
}

static void task_imu(void *) {
    TickType_t last = xTaskGetTickCount();
    TickType_t last_odom = xTaskGetTickCount();

    while (1) {
        imu_data_t d;

        taskENTER_CRITICAL(&g_state_mux);
        float spd = g_state.filtered_speed;
        taskEXIT_CRITICAL(&g_state_mux);
        imu_set_wheel_speed(spd);

        if (imu_update(&d)) {
            bool imu_cal_ready =
#if IMU_CAL_REQUIRE_HEATER_READY
                imu_heater_ready();
#else
                true;
#endif

            if (!s_bias_calibrated && imu_cal_ready && imu_is_static()) {
                imu_calibrate_bias();
                s_bias_calibrated = imu_bias_ready();
            }

            // Auto level calibration: accumulate static accel, tolerate brief interruptions.
            // Samples are collected WITHOUT level offset (raw tilt).
            if (!s_level_calibrated && imu_cal_ready) {
                if (s_level_count == 0 && s_level_skip == 0) {
                    imu_set_level_offset(0.0f, 0.0f, 0.0f);
                }
                if (imu_is_static()) {
                    s_level_skip = 0;
                    s_level_sum[0] += d.accelX;
                    s_level_sum[1] += d.accelY;
                    s_level_sum[2] += d.accelZ;
                    s_level_count++;
                    if (s_level_count >= LEVEL_CAL_SAMPLES) {
                        float ox = s_level_sum[0] / s_level_count;
                        float oy = s_level_sum[1] / s_level_count;
                        float oz = s_level_sum[2] / s_level_count - 1.0f;
                        imu_set_level_offset(ox, oy, oz);
                        if (!nvs_level_save(ox, oy, oz)) {
                            serial_tx_printf("ERROR: level cal active, NVS save failed\n");
                        }
                        s_level_calibrated = true;
                    }
                } else {
                    s_level_skip++;
                    if (s_level_skip > LEVEL_CAL_SKIP_MAX) {
                        s_level_sum[0] = s_level_sum[1] = s_level_sum[2] = 0.0f;
                        s_level_count = 0;
                        s_level_skip  = 0;
                    }
                }
            }

            float mx = 0.0f, my = 0.0f, mz = 0.0f;
            bool mag_cal_active = false;
            taskENTER_CRITICAL(&g_state_mux);
            mag_cal_active = g_state.mag_cal_running;
            taskEXIT_CRITICAL(&g_state_mux);
            bool mag_available = false;
            taskENTER_CRITICAL(&g_state_mux);
            mag_available = g_state.mag_available;
            taskEXIT_CRITICAL(&g_state_mux);
            bool mag_ok = mag_available && !mag_cal_active && s_mag.readCalibrated(mx, my, mz);

            s_madgwick.updateIMU(
                d.gyroX, d.gyroY, d.gyroZ,
                d.accelX, d.accelY, d.accelZ
            );

            float qw = s_madgwick.getQuatW();
            float qx = s_madgwick.getQuatX();
            float qy = s_madgwick.getQuatY();
            float qz = s_madgwick.getQuatZ();

            float accel[3] = {
                d.accelX * 9.80665f,
                d.accelY * 9.80665f,
                d.accelZ * 9.80665f
            };

            float gyro[3] = {
                d.gyroX * 0.0174533f,
                d.gyroY * 0.0174533f,
                d.gyroZ * 0.0174533f
            };

            TickType_t now_t = xTaskGetTickCount();
            float dt = (float)(now_t - last_odom) *
                       portTICK_PERIOD_MS / 1000.0f;

            if (mag_ok) {
                // Tilt-compensated magnetic heading is diagnostic only. Indoor magnetic
                // disturbances should not alter odometry yaw.
                float roll  = atan2f(2.0f*(qw*qx + qy*qz),
                                     1.0f - 2.0f*(qx*qx + qy*qy));
                float pitch = asinf(CONSTRAIN(2.0f*(qw*qy - qz*qx), -1.0f, 1.0f));

                float mx_h =  mx*cosf(pitch) + mz*sinf(pitch);
                float my_h =  mx*sinf(roll)*sinf(pitch)
                            + my*cosf(roll)
                            - mz*sinf(roll)*cosf(pitch);
                float mag_hdg_raw = atan2f(-my_h, mx_h);

                // EMA low-pass on mag heading (tau=0.5s) to suppress sensor noise
                if (!s_mag_hdg_ema_init) {
                    s_mag_hdg_ema      = mag_hdg_raw;
                    s_mag_hdg_ema_init = true;
                } else {
                    float herr = mag_hdg_raw - s_mag_hdg_ema;
                    herr = wrap_pi(herr);
                    float mag_tau = imu_is_static() ?
                                    MAG_HEADING_EMA_TAU_STATIC_S :
                                    MAG_HEADING_EMA_TAU_MOVING_S;
                    float mag_alpha = CONSTRAIN(dt / mag_tau, 0.0f, 1.0f);
                    s_mag_hdg_ema += herr * mag_alpha;
                    s_mag_hdg_ema = wrap_pi(s_mag_hdg_ema);
                }
                float mag_hdg = s_mag_hdg_ema;

                float hdg_raw_deg = mag_hdg * 180.0f / 3.14159265f;
                if (hdg_raw_deg < 0.0f) hdg_raw_deg += 360.0f;

                taskENTER_CRITICAL(&g_state_mux);
                g_state.compass_heading = hdg_raw_deg;
                g_state.mag_heading_raw = hdg_raw_deg;
                g_state.yaw_cf_err      = 0.0f;
                g_state.yaw_drift_rate  = s_yaw_drift_rate;
                g_state.mag_ok_count++;
                taskEXIT_CRITICAL(&g_state_mux);
            }

            taskENTER_CRITICAL(&g_state_mux);

            g_state.accel[0] = accel[0];
            g_state.accel[1] = accel[1];
            g_state.accel[2] = accel[2];

            g_state.gyro[0] = gyro[0];
            g_state.gyro[1] = gyro[1];
            g_state.gyro[2] = gyro[2];

            if (mag_ok) {
                g_state.mag[0] = mx;
                g_state.mag[1] = my;
                g_state.mag[2] = mz;
            }

            g_state.is_static = imu_is_static();
            g_state.temp = d.temp;

            float fx = qw * qw + qx * qx - qy * qy - qz * qz;
            float fy = 2.0f * (qw * qz + qx * qy);
            float raw_yaw = atan2f(fy, fx);
            float odom_yaw = wrap_pi(raw_yaw - s_odom_yaw_zero);
            float rel_quat[4];
            yaw_relative_quat(s_odom_yaw_zero, qw, qx, qy, qz, rel_quat);

            g_state.imu_count++;
            g_state.quat[0] = rel_quat[0];
            g_state.quat[1] = rel_quat[1];
            g_state.quat[2] = rel_quat[2];
            g_state.quat[3] = rel_quat[3];
            g_state.imu_yaw_raw = raw_yaw;
            g_state.odom_yaw = odom_yaw;

            if (dt > 0.001f && dt < 0.1f) {
                float spd = g_state.odom_speed;
                float odom_fx = cosf(odom_yaw);
                float odom_fy = sinf(odom_yaw);

                g_state.odom_vel[0] = spd;
                g_state.odom_vel[1] = 0.0f;
                g_state.odom_vel[2] = 0.0f;

                g_state.odom_pos[0] += spd * odom_fx * dt;
                g_state.odom_pos[1] += spd * odom_fy * dt;
            }

            taskEXIT_CRITICAL(&g_state_mux);

            last_odom = now_t;
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
    }
}

static void task_control(void *) {
    TickType_t last = xTaskGetTickCount();
    TickType_t last_spd_t = xTaskGetTickCount();
    TickType_t last_ctrl_t = xTaskGetTickCount();

    int32_t last_count = 0;
    unsigned long last_volt_ms = 0;

    while (1) {
        TickType_t now_t = xTaskGetTickCount();
        float ctrl_dt_sec = (float)(now_t - last_ctrl_t) *
                            portTICK_PERIOD_MS / 1000.0f;
        if (ctrl_dt_sec <= 0.0f) {
            ctrl_dt_sec = (float)CONTROL_INTERVAL / 1000.0f;
        }
        last_ctrl_t = now_t;

        float dt_ms = (float)(now_t - last_spd_t) * portTICK_PERIOD_MS;

        if (dt_ms >= (float)SPEED_CALC_INTERVAL) {
            int32_t cnt = encoder_get_count();
            int32_t delta = cnt - last_count;

            float ratio = (float)ENCODER_PPR * GEAR_RATIO;
            float rps = (float)delta / ratio / (dt_ms / 1000.0f);
            float raw_spd = rps * WHEEL_CIRCUMFERENCE;

            taskENTER_CRITICAL(&g_state_mux);
            raw_spd *= g_state.odom_scale;
            g_state.filtered_speed =
                SPEED_LPF_ALPHA * raw_spd +
                (1.0f - SPEED_LPF_ALPHA) * g_state.filtered_speed;
            g_state.odom_speed =
                ODOM_SPEED_LPF_ALPHA * raw_spd +
                (1.0f - ODOM_SPEED_LPF_ALPHA) * g_state.odom_speed;
            taskEXIT_CRITICAL(&g_state_mux);

            last_count = cnt;
            last_spd_t = now_t;
        }

        float target;
        float spd;
        int steer;
        int deadband_us;

        taskENTER_CRITICAL(&g_state_mux);
        target = g_state.target_speed;
        spd = g_state.filtered_speed;
        steer = g_state.steering_pulse;
        deadband_us = g_state.speed_deadband_us;
        taskEXIT_CRITICAL(&g_state_mux);

        int throttle;
        safety_snapshot_t safety = safety_event_snapshot();

        if (!safety.allow_motion) {
            throttle = THROTTLE_NEUTRAL;
            target = 0.0f;
            taskENTER_CRITICAL(&g_state_mux);
            g_state.target_speed = 0.0f;
            pid_reset(&g_pid);
            taskEXIT_CRITICAL(&g_state_mux);
        } else if (fabsf(target) < SPEED_STOP_THRESHOLD) {
            throttle = THROTTLE_NEUTRAL;
            taskENTER_CRITICAL(&g_state_mux);
            pid_reset(&g_pid);
            taskEXIT_CRITICAL(&g_state_mux);
        } else {
            int ff_neutral = throttle_feedforward_neutral(target, deadband_us);
            taskENTER_CRITICAL(&g_state_mux);
            throttle = pid_update(
                &g_pid,
                target,
                spd,
                ff_neutral,
                ctrl_dt_sec
            );
            taskEXIT_CRITICAL(&g_state_mux);
        }

        servo_set_pulse(LEDC_CH_THROTTLE, throttle);
        servo_set_pulse(LEDC_CH_STEERING, steer);

        unsigned long now_ms =
            (unsigned long)(esp_timer_get_time() / 1000);

        if (now_ms - last_volt_ms >= 500) {
            int raw = 0;
            if (adc_oneshot_read(s_adc, ADC_CHANNEL_3, &raw) == ESP_OK) {
                float voltage = adc_to_voltage(raw);

                taskENTER_CRITICAL(&g_state_mux);
                g_state.voltage = voltage;
                taskEXIT_CRITICAL(&g_state_mux);

            }

            last_volt_ms = now_ms;
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(CONTROL_INTERVAL));
    }
}

static void task_comm(void *) {
    char line_buf[1024];
    int line_len = 0;

    unsigned long last_imu_ms = 0;
    unsigned long last_sync_ms = 0;
    unsigned long last_mag_ms = 0;
    unsigned long last_odom_ms = 0;
    unsigned long last_rc_ms = 0;
    unsigned long last_battery_ms = 0;
    unsigned long last_rc_ok_ms = 0;

    bool rc_initialized = false;

    while (1) {
        unsigned long now_ms =
            (unsigned long)(esp_timer_get_time() / 1000);

        while (1) {
            uint8_t rx_buf[128];
            int rx_len = serial_port_read_bytes(rx_buf, sizeof(rx_buf), 0);
            if (rx_len <= 0) {
                break;
            }

            for (int i = 0; i < rx_len; i++) {
                char c = (char)rx_buf[i];

                if (c == '\n') {
                    if (line_len > 0) {
                        while (line_len > 0 &&
                               (line_buf[line_len - 1] == '\r' ||
                                line_buf[line_len - 1] == ' ')) {
                            line_len--;
                        }
                        if (line_len > 0) {
                            line_buf[line_len] = '\0';
                            serial_cmd_process(line_buf);
                        }
                        line_len = 0;
                    }
                } else if (line_len < (int)sizeof(line_buf) - 1) {
                    line_buf[line_len++] = c;
                } else {
                    line_len = 0;
                    serial_tx_printf("ERROR: command too long\n");
                }
            }
        }

        if (fw_update_active()) {
            taskENTER_CRITICAL(&g_state_mux);
            g_state.target_speed = 0.0f;
            g_state.steering_pulse = steering_trimmed_pulse(STEERING_CENTER, g_state.steering_trim_deg);
            g_state.remote_active = false;
            g_state.serial_control_active = true;
            taskEXIT_CRITICAL(&g_state_mux);
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        bool serial_control_active = false;
        bool remote_active = false;
        taskENTER_CRITICAL(&g_state_mux);
        serial_control_active = g_state.serial_control_active;
        remote_active = g_state.remote_active;
        taskEXIT_CRITICAL(&g_state_mux);

        if (serial_control_active && !remote_active) {
            // Re-sample after serial_cmd_process() may have updated
            // last_serial_cmd_ms; using the loop-top now_ms causes unsigned
            // wrap-around and fires the timeout immediately after each command.
            unsigned long serial_now_ms =
                (unsigned long)(esp_timer_get_time() / 1000);
            taskENTER_CRITICAL(&g_state_mux);
            unsigned long last_cmd = g_state.last_serial_cmd_ms;
            taskEXIT_CRITICAL(&g_state_mux);
            if (serial_now_ms - last_cmd > (unsigned long)SERIAL_TIMEOUT) {
                taskENTER_CRITICAL(&g_state_mux);
                g_state.target_speed = 0.0f;
                g_state.steering_pulse = steering_trimmed_pulse(STEERING_CENTER, g_state.steering_trim_deg);
                taskEXIT_CRITICAL(&g_state_mux);
            }
        }

        sbus_data_t rc;

        if (sbus_read(&rc)) {
            last_rc_ok_ms = now_ms;
            rc_initialized = true;

            if (rc.failsafe || rc.lost_frame) {
                if (!g_state.failsafe) {
                    taskENTER_CRITICAL(&g_state_mux);
                    g_state.failsafe = true;
                    g_state.remote_active = false;
                    g_state.serial_control_active = true;
                    g_state.target_speed = 0.0f;
                    g_state.steering_pulse = steering_trimmed_pulse(STEERING_CENTER, g_state.steering_trim_deg);
                    taskEXIT_CRITICAL(&g_state_mux);

                }
            } else {
                taskENTER_CRITICAL(&g_state_mux);
                g_state.failsafe = false;
                for (int i = 0; i < 10; i++) {
                    g_state.rc_ch[i] = rc.ch[i];
                }
                taskEXIT_CRITICAL(&g_state_mux);

                if (rc.ch[CONTROL_MODE_CHANNEL] < 1500) {
                    if (!g_state.remote_active) {
                        taskENTER_CRITICAL(&g_state_mux);
                        g_state.remote_active = true;
                        g_state.serial_control_active = false;
                        taskEXIT_CRITICAL(&g_state_mux);
                    }
                } else {
                    if (g_state.remote_active) {
                        taskENTER_CRITICAL(&g_state_mux);
                        g_state.remote_active = false;
                        g_state.serial_control_active = true;
                        g_state.target_speed = 0.0f;
                        g_state.steering_pulse = steering_trimmed_pulse(STEERING_CENTER, g_state.steering_trim_deg);
                        taskEXIT_CRITICAL(&g_state_mux);

                        pid_reset(&g_pid);
                    }
                }

                if (g_state.remote_active) {
                    int steer_raw = map_range(
                        rc.ch[STEERING_CHANNEL],
                        SBUS_MIN,
                        SBUS_MAX,
                        STEERING_MIN,
                        STEERING_MAX
                    );

                    steer_raw = apply_deadzone(
                        steer_raw,
                        STEERING_CENTER,
                        RC_STEERING_DEADZONE
                    );

                    if (STEERING_REVERSE) {
                        steer_raw =
                            STEERING_MAX -
                            (steer_raw - STEERING_MIN);
                    }

                    steer_raw = iclamp(
                        steer_raw,
                        STEERING_MIN,
                        STEERING_MAX
                    );

                    int thr_raw = map_range(
                        rc.ch[THROTTLE_CHANNEL],
                        SBUS_MIN,
                        SBUS_MAX,
                        THROTTLE_MIN,
                        THROTTLE_MAX
                    );

                    thr_raw = apply_deadzone(
                        thr_raw,
                        THROTTLE_NEUTRAL,
                        RC_THROTTLE_DEADZONE
                    );

                    if (THROTTLE_REVERSE) {
                        thr_raw =
                            THROTTLE_MAX -
                            (thr_raw - THROTTLE_MIN);
                    }

                    float spd = 0.0f;

                    if (thr_raw > THROTTLE_NEUTRAL) {
                        spd =
                            (float)(thr_raw - THROTTLE_NEUTRAL) /
                            (float)(THROTTLE_MAX - THROTTLE_NEUTRAL) *
                            MAX_SPEED_FORWARD;
                    } else if (thr_raw < THROTTLE_NEUTRAL) {
                        spd =
                            (float)(thr_raw - THROTTLE_NEUTRAL) /
                            (float)(THROTTLE_NEUTRAL - THROTTLE_MIN) *
                            (-MAX_SPEED_REVERSE);
                    }

                    bool full =
                        (rc.ch[SPEED_MODE_CHANNEL] >= 1500);

                    if (!full) {
                        spd *= RC_LOW_SPEED_SCALE;
                    }

                    taskENTER_CRITICAL(&g_state_mux);
                    g_state.speed_full_mode = full;
                    g_state.target_speed = spd;
                    g_state.steering_pulse = steering_trimmed_pulse(steer_raw, g_state.steering_trim_deg);
                    taskEXIT_CRITICAL(&g_state_mux);
                }
            }
        } else if (rc_initialized && g_state.remote_active) {
            if (now_ms - last_rc_ok_ms >= 200) {
                taskENTER_CRITICAL(&g_state_mux);
                g_state.failsafe = true;
                g_state.remote_active = false;
                g_state.serial_control_active = true;
                g_state.target_speed = 0.0f;
                g_state.steering_pulse = steering_trimmed_pulse(STEERING_CENTER, g_state.steering_trim_deg);
                taskEXIT_CRITICAL(&g_state_mux);

            }
        }

        bool stream_enabled = false;
        bool stream_legacy = false;
        taskENTER_CRITICAL(&g_state_mux);
        stream_enabled = g_state.stream_enabled;
        stream_legacy = g_state.stream_legacy;
        taskEXIT_CRITICAL(&g_state_mux);

        if (stream_enabled) {
            if (!stream_legacy && now_ms - last_sync_ms >= (unsigned long)SYNC_SEND_INTERVAL) {
                serial_frame_send_sync();
                last_sync_ms = now_ms;
            }

            if (stream_legacy && now_ms - last_odom_ms >= (unsigned long)ODOM_SEND_INTERVAL) {
                serial_frame_send_odom();
                last_odom_ms = now_ms;
            }

            if (stream_legacy && now_ms - last_imu_ms >= (unsigned long)IMU_SEND_INTERVAL) {
                serial_frame_send_imu();
                last_imu_ms = now_ms;
            }

            if (now_ms - last_mag_ms >= (unsigned long)MAG_SEND_INTERVAL) {
                serial_frame_send_mag();
                last_mag_ms = now_ms;
            }

            if (now_ms - last_rc_ms >= (unsigned long)RC_SEND_INTERVAL) {
                send_rc_frame();
                last_rc_ms = now_ms;
            }

            if (now_ms - last_battery_ms >= (unsigned long)BATTERY_SEND_INTERVAL) {
                serial_frame_send_battery();
                last_battery_ms = now_ms;
            }

        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

extern "C" void app_main(void) {
    memset(&g_state, 0, sizeof(g_state));

    g_state.quat[0] = 1.0f;
    g_state.odom_scale = ODOM_SCALE_DEFAULT;
    g_state.steering_trim_deg = STEERING_TRIM_DEFAULT_DEG;
    g_state.steering_pulse = STEERING_CENTER;
    g_state.serial_control_active = true;
    g_state.failsafe = true;
    g_state.kp = PID_KP_DEFAULT;
    g_state.ki = PID_KI_DEFAULT;
    g_state.kd = PID_KD_DEFAULT;
    g_state.speed_deadband_us = THROTTLE_FF_DEADBAND_DEFAULT_US;
    g_state.cf_alpha_static = COMPASS_CF_ALPHA_STATIC;
    g_state.cf_alpha_moving = COMPASS_CF_ALPHA_MOVING;
    g_state.cf_speed_thr    = COMPASS_CF_SPEED_THR;
    g_state.stream_enabled = true;
    g_state.stream_legacy = false;

    serial_cmd_init();
    gpio_config_t io_cfg = {};
    io_cfg.pin_bit_mask = (1ULL << IO_CTRL_BAT);
    io_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_cfg);

    gpio_set_level((gpio_num_t)IO_CTRL_BAT, 0);

    nvs_params_init();
    nvs_params_load(&g_state.kp, &g_state.ki, &g_state.kd);
    nvs_cf_load(&g_state.cf_alpha_static, &g_state.cf_alpha_moving, &g_state.cf_speed_thr);
    nvs_chassis_load(&g_state.odom_scale, &g_state.steering_trim_deg);
    nvs_speed_deadband_load(&g_state.speed_deadband_us);
    g_state.steering_pulse = steering_trimmed_pulse(STEERING_CENTER, g_state.steering_trim_deg);

    {
        float ox, oy, oz;
        if (nvs_level_load(&ox, &oy, &oz)) {
            imu_set_level_offset(ox, oy, oz);
        }
        s_level_calibrated = false;
    }

    pid_init(
        &g_pid,
        g_state.kp,
        g_state.ki,
        g_state.kd,
        PID_MAX_INTEGRAL,
        PID_DEADBAND
    );

    i2c_master_bus_handle_t i2c_bus;

    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = (gpio_num_t)I2C_SDA;
    bus_cfg.scl_io_num = (gpio_num_t)I2C_SCL;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;

    i2c_new_master_bus(&bus_cfg, &i2c_bus);

    i2c_device_config_t imu_dev_cfg = {};
    imu_dev_cfg.device_address = IMU_ADDRESS;
    imu_dev_cfg.scl_speed_hz = 400000;

    i2c_master_dev_handle_t imu_dev;
    i2c_master_bus_add_device(i2c_bus, &imu_dev_cfg, &imu_dev);

    i2c_device_config_t mag_dev_cfg = {};
    mag_dev_cfg.device_address = QMC6309_ADDR;
    mag_dev_cfg.scl_speed_hz = 400000;

    i2c_master_dev_handle_t mag_dev;
    i2c_master_bus_add_device(i2c_bus, &mag_dev_cfg, &mag_dev);

    imu_init(imu_dev);

    s_mag.setDevice(mag_dev);

    bool mag_available = s_mag.begin();
    taskENTER_CRITICAL(&g_state_mux);
    g_state.mag_available = mag_available;
    taskEXIT_CRITICAL(&g_state_mux);
    if (!mag_available) {
        FW_LOGF("WARN: QMC6309 magnetometer unavailable\n");
    }

    {
        float hard[3], soft[9];
        bool mag_calibrated = nvs_mag_load(hard, soft);
        s_mag.setCalibration(hard, soft);
        taskENTER_CRITICAL(&g_state_mux);
        g_state.mag_calibrated = mag_calibrated;
        taskEXIT_CRITICAL(&g_state_mux);
    }

    s_madgwick.begin(0.1f);

    encoder_init();
    servo_init();

    servo_set_pulse(LEDC_CH_THROTTLE, THROTTLE_NEUTRAL);
    servo_set_pulse(LEDC_CH_STEERING, g_state.steering_pulse);

    buzzer_init();
    alert_buzzer_init();
    melody_player_init();
    led_init();
    status_indicator_init();
    imu_heater_init();
    sbus_init();

    adc_oneshot_unit_init_cfg_t adc_cfg = {};
    adc_cfg.unit_id = ADC_UNIT_1;
    adc_oneshot_new_unit(&adc_cfg, &s_adc);

    adc_oneshot_chan_cfg_t ch_cfg = {};
    ch_cfg.atten = ADC_ATTEN_DB_12;
    ch_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;

    adc_oneshot_config_channel(s_adc, ADC_CHANNEL_3, &ch_cfg);

    static const uint32_t scale[] = {262, 330, 392, 523};

    for (int i = 0; i < 4; i++) {
        buzzer_tone(scale[i], 150);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    xTaskCreatePinnedToCore(task_imu, "imu", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(task_control, "ctrl", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(task_comm, "comm", 8192, NULL, 3, NULL, 0);

    fw_update_mark_app_valid();

    FW_LOGF("INFO: osrcore ready\n");

    vTaskDelete(NULL);
}
