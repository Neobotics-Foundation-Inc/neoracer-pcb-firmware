#pragma once

#include <stdint.h>
#include "sdkconfig.h"

#define STRINGIFY_VALUE(x) #x
#define STRINGIFY(x) STRINGIFY_VALUE(x)

// ==================== 传感器选择 ====================
#define USE_QMI8658
// #define USE_ICM45686

// ==================== IMU零偏抑制参数 ====================
#define GYRO_BIAS_SAMPLES            100
#define GYRO_BIAS_INIT_ABS_MAX       8.0f  // dps: reject impossible startup bias
#define GYRO_BIAS_INIT_STD_MAX       0.8f  // dps: startup bias samples must be quiet
#define GYRO_BIAS_UPDATE_THRESHOLD   0.005f
#define GYRO_BIAS_UPDATE_ALPHA       0.25f
#define GYRO_BIAS_UPDATE_GYR_MAX     0.8f
#define MOVEMENT_DETECTION_THRESHOLD 0.02f
#define ACCEL_MOVEMENT_THRESHOLD     0.05f
#define STATIONARY_COUNT_THRESHOLD   10
#define GYRO_BIAS_BATCH_SAMPLES      100   // ~0.5s at 200Hz, fast static drift tracking
#define STATIC_GYR_NORM_MAX          3.0f  // dps: tolerate raw gyro bias before initial calibration
#define STATIC_GYR_INIT_NORM_MAX     20.0f // dps: reject obvious hand rotation before bias exists
#define STATIC_GYR_STD_MAX           1.2f  // dps: short-window gyro noise must be small when static
#define STATIC_ACC_STD_MAX           0.05f // g: short-window accel noise must be small when static
#define STATIC_AMAG_TOL              0.25f // g: static accel norm must stay near 1g
#define STATIC_WHEEL_SPEED_MAX       0.03f // m/s: reject rolling while tolerating encoder noise
#define YAW_DRIFT_STATIC_TAU_S       8.0f  // faster residual Z drift convergence while static
#define IMU_CAL_REQUIRE_HEATER_READY 0     // allow startup bias/level calibration before heater ready

// ==================== 编码器参数 ====================
#define EA                  CONFIG_OSCORE_ENCODER_A_GPIO
#define EB                  CONFIG_OSCORE_ENCODER_B_GPIO
#define ENCODER_PPR         CONFIG_OSCORE_ENCODER_PPR
#define GEAR_RATIO          ((float)CONFIG_OSCORE_GEAR_RATIO_MILLI / 1000.0f)
#define WHEEL_RADIUS        ((float)CONFIG_OSCORE_WHEEL_RADIUS_UM / 1000000.0f)
#define WHEEL_CIRCUMFERENCE (2.0f * 3.14159265358979f * WHEEL_RADIUS)
#define SPEED_CALC_INTERVAL CONFIG_OSCORE_SPEED_CALC_INTERVAL_MS // ms

// ==================== 速度PID ====================
#define PID_KP_DEFAULT    447.0f
#define PID_KI_DEFAULT    4.7f
#define PID_KD_DEFAULT    4.7f
#define PID_PROFILE_VERSION 2
#define PID_MAX_INTEGRAL  1000.0f
#define PID_DEADBAND      0.05f
#define CONTROL_INTERVAL  20     // ms
#define SPEED_LPF_ALPHA   0.15f  // low-pass weight for new speed sample
#define ODOM_SPEED_LPF_ALPHA 0.95f // fast encoder speed for odometry integration
#define THROTTLE_FF_DEADBAND_DEFAULT_US 90  // minimum drive pulse beyond neutral for friction/deadzone
#define THROTTLE_FF_DEADBAND_MIN_US     0
#define THROTTLE_FF_DEADBAND_MAX_US     300
#define THROTTLE_FF_GAIN_US_PER_MPS 68.0f  // throttle feed-forward slope in PWM us per m/s
#define THROTTLE_FF_MAX_US 500
#define MAX_SPEED_FORWARD  6.0f
#define MAX_SPEED_REVERSE -6.0f
#define RC_LOW_SPEED_PERCENT 15
#define RC_LOW_SPEED_SCALE   ((float)RC_LOW_SPEED_PERCENT / 100.0f)

// ==================== SBUS ====================
#define STEERING_CHANNEL      0
#define THROTTLE_CHANNEL      2
#define CONTROL_MODE_CHANNEL  6
#define SPEED_MODE_CHANNEL    7

#define SBUS_MIN 240
#define SBUS_MAX 1810

#define STEERING_MIN    1000
#define STEERING_MAX    2000
#define STEERING_CENTER 1500
#define STEERING_MAX_ANGLE_DEG 30.0f
#define STEERING_TRIM_DEFAULT_DEG 0.0f
#define STEERING_TRIM_MIN_DEG    -5.0f
#define STEERING_TRIM_MAX_DEG     5.0f
#define STEERING_TRIM_LEGACY_MIN_US    -50
#define STEERING_TRIM_LEGACY_MAX_US     50

#define ODOM_SCALE_DEFAULT 1.0f
#define ODOM_SCALE_MIN     0.5f
#define ODOM_SCALE_MAX     1.5f

#define THROTTLE_MIN     1000
#define THROTTLE_MAX     2000
#define THROTTLE_NEUTRAL 1500

#define RC_STEERING_DEADZONE 10
#define RC_THROTTLE_DEADZONE 30

#define STEERING_REVERSE true
#define THROTTLE_REVERSE false

#define SPEED_STOP_THRESHOLD 0.05f

// ==================== IMU 温控加热 ====================
#define IMU_HEATER_PIN              46
#define IMU_HEATER_TARGET_C         56.0f   // bang-bang setpoint (above self-heat floor ~54°C)
#define IMU_HEATER_PREHEAT_MARGIN   8.0f    // °C below target: full-on preheat range
#define IMU_HEATER_WARM_C           38.0f   // milestone 1: warm enough to use
#define IMU_HEATER_STABLE_C         54.0f   // milestone 2: thermal steady state
#define IMU_HEATER_READY_TIMEOUT    300000  // ms: fallback if slope never settles
#define IMU_HEATER_SLOPE_WINDOW     6       // samples (6 × 500ms = 3s slope window)
#define IMU_HEATER_SLOPE_MAX        0.6f    // °C/sample: tolerate cool-down after reaching target band

// ==================== 引脚定义 ====================
#define THROTTLE_PIN    1
#define STEERING_PIN    2
#define VOLTAGE_PIN     4
#define IO_CTRL_BAT     16
#define BUZZER_PIN      42
#define ALERT_BUZZER_PIN 47
#define LED_PIN         45
#define NUM_LEDS        1
#define I2C_SDA         10
#define I2C_SCL         11

// SBUS: UART0, GPIO43=TXD0, GPIO44=RXD0
#define SBUS_UART_NUM   0
#define SBUS_RX_PIN     44
#define SBUS_TX_PIN     43

#ifdef USE_ICM45686
#define IMU_INT_PIN 21
#endif

#ifdef USE_QMI8658
#define IMU_ADDRESS 0x6B
#endif

// ==================== PWM ====================
#define PWM_FREQUENCY  50
#define PWM_RESOLUTION 14

// LEDC channel 分配
#define LEDC_CH_THROTTLE 0
#define LEDC_CH_STEERING 1
#define LEDC_CH_BUZZER   2

// ==================== 电压检测 ====================
#define LOW_VOLTAGE_THRESHOLD       10.8f  // 3S LiPo: 3.6V/cell sustained low-voltage alarm
#define LOW_VOLTAGE_RECOVER         11.1f  // 3S LiPo: recovery hysteresis to avoid alarm chatter
#define LOW_VOLTAGE_CONFIRM_MS      3000   // require sustained sag before entering alarm
#define LOW_VOLTAGE_RECOVER_MS      3000   // require sustained recovery before leaving alarm
#define LOW_VOLTAGE_ALERT_PERIOD_MS 5000   // repeat active buzzer warning while alarm is latched
#define NUM_ADC_POINTS              9

// ==================== 诊断日志 ====================
#define FW_VERBOSE_LOG 0  // keep protocol clean by default; set to 1 for boot/debug logs

// ==================== 数据发送间隔 ====================
#define IMU_SEND_INTERVAL  5    // ms
#define SYNC_SEND_INTERVAL 5    // ms
#define ODOM_SEND_INTERVAL 20   // ms
#define MAG_SEND_INTERVAL  50   // ms
#define RC_SEND_INTERVAL   100  // ms
#define BATTERY_SEND_INTERVAL 2000 // ms

// ==================== 串口协议 ====================
#define SERIAL_TIMEOUT 500   // ms

// ==================== NVS ====================
#define PREF_NAMESPACE "pid_params"
#define PREF_KEY_KP    "kp"
#define PREF_KEY_KI    "ki"
#define PREF_KEY_KD    "kd"
#define PREF_KEY_INIT  "init"
#define PREF_KEY_PID_PROFILE "profile"

#define PREF_NS_MAG       "mag_calib"
#define PREF_KEY_MAG_HI   "hi"    // hard-iron: 3×float blob
#define PREF_KEY_MAG_SI   "si"    // soft-iron: 9×float blob (row-major 3×3)
#define PREF_KEY_MAG_INIT "init"

// ==================== 罗盘互补滤波 ====================
#define COMPASS_CF_ALPHA_STATIC  0.999f  // static: slow mag pullback, suppress heading jitter
#define COMPASS_CF_ALPHA_MOVING  0.995f  // moving: reject motor EMI (τ ≈ 1s)
#define COMPASS_CF_SPEED_THR     0.5f    // m/s, speed threshold for moving mode
#define MAG_HEADING_EMA_TAU_STATIC_S 5.0f
#define MAG_HEADING_EMA_TAU_MOVING_S 0.5f

#define PREF_NS_CF        "cf_params"
#define PREF_KEY_CF_AS    "alpha_s"
#define PREF_KEY_CF_AM    "alpha_m"
#define PREF_KEY_CF_THR   "spd_thr"
#define PREF_KEY_CF_INIT  "init"

// ==================== 水平校准 NVS ====================
#define LEVEL_CAL_SAMPLES   100  // samples needed (at ~40Hz output = 2.5s)
#define LEVEL_CAL_SKIP_MAX   10  // tolerate up to 10 non-static frames without resetting
#define PREF_NS_LEVEL       "level_cal"
#define PREF_KEY_LEVEL_OX   "ox"
#define PREF_KEY_LEVEL_OY   "oy"
#define PREF_KEY_LEVEL_OZ   "oz"
#define PREF_KEY_LEVEL_INIT "init"

// ==================== 底盘客户标定 NVS ====================
#define PREF_NS_CHASSIS          "chassis_cal"
#define PREF_KEY_ODOM_SCALE      "odom_scale"
#define PREF_KEY_STEERING_TRIM_DEG "steer_trim_deg"
#define PREF_KEY_STEERING_TRIM_US  "steer_trim"
#define PREF_KEY_CHASSIS_INIT    "init"

#define PREF_NS_SPEED           "speed_cal"
#define PREF_KEY_SPEED_DEADBAND "deadband_us"
#define PREF_KEY_SPEED_INIT     "init"

// ==================== 工具宏 ====================
#define CONSTRAIN(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define MAP_FLOAT(x, in_min, in_max, out_min, out_max) \
    ((float)((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))
