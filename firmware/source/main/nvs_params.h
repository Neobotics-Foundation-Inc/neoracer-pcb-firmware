#pragma once
#include <stdbool.h>

void nvs_params_init(void);
void nvs_params_load(float *kp, float *ki, float *kd);
bool nvs_params_save(float kp, float ki, float kd);
bool nvs_params_save_async(float kp, float ki, float kd);

// hard[3]: hard-iron offset (Gauss)
// soft[9]: soft-iron matrix, row-major 3×3; identity = {1,0,0, 0,1,0, 0,0,1}
bool nvs_mag_save(const float hard[3], const float soft[9]);
bool nvs_mag_save_async(const float hard[3], const float soft[9]);
bool nvs_mag_load(float hard[3], float soft[9]);

bool nvs_cf_save(float alpha_static, float alpha_moving, float speed_thr);
void nvs_cf_load(float *alpha_static, float *alpha_moving, float *speed_thr);

// ox, oy, oz: accel mounting offsets in g (oz = az_mean - 1.0)
bool nvs_level_save(float ox, float oy, float oz);
bool nvs_level_load(float *ox, float *oy, float *oz);
bool nvs_level_reset(void);
bool nvs_level_reset_async(void);

bool nvs_chassis_save(float odom_scale, float steering_trim_deg);
bool nvs_chassis_save_async(float odom_scale, float steering_trim_deg);
void nvs_chassis_load(float *odom_scale, float *steering_trim_deg);

bool nvs_speed_deadband_save(int deadband_us);
bool nvs_speed_deadband_save_async(int deadband_us);
void nvs_speed_deadband_load(int *deadband_us);
