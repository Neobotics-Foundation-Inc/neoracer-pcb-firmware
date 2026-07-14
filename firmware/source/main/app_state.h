#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float target_speed;    // m/s, written by comm task
    float filtered_speed;  // m/s, written by control task
    float odom_speed;      // m/s, fast encoder speed for odometry integration
    float odom_scale;      // encoder speed/distance scale, customer-calibrated
    float kp, ki, kd;
    int speed_deadband_us; // throttle feed-forward deadband around neutral, in PWM microseconds
    float odom_pos[3];     // m
    float odom_vel[3];     // m/s
    float odom_yaw;        // rad, yaw-zero relative yaw derived from Madgwick
    float imu_yaw_raw;     // rad, raw IMU/Madgwick yaw diagnostic mirror
    float quat[4];         // w x y z, Madgwick orientation with odom yaw-zero applied
    float accel[3];        // m/s² (converted from g)
    float gyro[3];         // rad/s (converted from deg/s)
    float mag[3];          // Gauss
    float compass_heading; // deg, 0-360, tilt-compensated absolute magnetic heading
    float mag_heading_raw; // deg, 0-360, tilt-compensated mag heading before CF (diagnostic)
    float yaw_cf_err;       // rad, reserved mag-vs-yaw diagnostic; 0 when mag fusion is disabled
    float yaw_drift_rate;   // rad/s, reserved residual gyro Z drift diagnostic
    uint32_t mag_ok_count; // total mag_ok=true hits since boot (diagnostic)
    uint32_t imu_count;    // total IMU updates since boot (diagnostic)
    bool mag_available;    // QMC6309 init succeeded; false means mag is diagnostic-unavailable
    float cf_alpha_static;   // CF alpha at rest  (default COMPASS_CF_ALPHA_STATIC)
    float cf_alpha_moving;   // CF alpha at speed (default COMPASS_CF_ALPHA_MOVING)
    float cf_speed_thr;      // m/s threshold     (default COMPASS_CF_SPEED_THR)
    float voltage;         // V
    float temp;            // °C (IMU die temperature)
    int   steering_pulse;  // µs, written by comm task
    float steering_trim_deg; // deg, added to steering center/commands
    int   rc_ch[10];          // last valid RC channel values (ch0-ch9)
    bool  remote_active;
    bool  failsafe;
    bool  is_static;
    bool  speed_full_mode;
    bool  serial_control_active;
    bool  mag_calibrated;          // true when hard/soft-iron calibration was loaded or saved
    bool  mag_cal_running;         // true while 'mc cal' is in progress
    bool  stream_enabled;          // periodic telemetry frames for host computer
    bool  stream_legacy;           // true: i/o/m/r/b, false: s/m/r/b
    unsigned long last_serial_cmd_ms;
} app_state_t;

extern app_state_t g_state;
