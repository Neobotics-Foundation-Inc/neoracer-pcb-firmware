#pragma once

void  imu_heater_init(void);
void  imu_heater_set_target(float temp_c);
float imu_heater_get_target(void);
bool  imu_heater_is_on(void);
bool  imu_heater_ready(void);   // true once slope is stable at target (56°C)
bool  imu_heater_warm(void);    // true once temp crossed WARM_C (38°C)
bool  imu_heater_stable(void);  // true once temp crossed STABLE_C (54°C)
