#pragma once
#include <stdbool.h>
#include "driver/i2c_master.h"

typedef struct {
    float accelX, accelY, accelZ;  // g, level-corrected
    float gyroX,  gyroY,  gyroZ;   // deg/s, bias-corrected
    float temp;                     // °C
} imu_data_t;

void imu_init(i2c_master_dev_handle_t dev);

// Blocking startup bias calibration — call once after IMU temperature is stable.
void imu_calibrate_bias(void);

// Returns true when new data is available. Applies bias and level correction internally.
bool imu_update(imu_data_t *out);

// Level calibration: offsets in g, applied as accel_corrected = accel_raw - offset.
// oz should be (az_mean - 1.0) so that accelZ reads 1.0g on flat ground.
void imu_set_level_offset(float ox, float oy, float oz);
void imu_get_level_offset(float *ox, float *oy, float *oz);

bool imu_is_static(void);
bool imu_bias_ready(void);
void imu_get_bias(float *bx, float *by, float *bz);  // deg/s
void imu_set_wheel_speed(float speed_mps);

// Trigger a fresh level calibration (zeros current offset, restarts accumulation).
void level_cal_trigger(void);
bool level_cal_is_calibrated(void);
