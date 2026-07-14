#include "imu_heater.h"
#include "app_state.h"
#include "config.h"
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include <math.h>

#define HEATER_HYSTERESIS 1.0f

static float s_target = IMU_HEATER_TARGET_C;
static bool  s_on     = false;
static bool  s_ready  = false;
static bool  s_warm   = false;
static bool  s_stable = false;

extern portMUX_TYPE g_state_mux;

// GPIO46 is a strapping pin; ESP-IDF gpio_config silently leaves output-enable unset.
// Drive it directly via registers to bypass the restriction.
#define HEATER_PIN_BIT  (1UL << (IMU_HEATER_PIN - 32))

static void heater_set(bool on) {
    s_on = on;
    if (on) {
        REG_WRITE(GPIO_OUT1_W1TS_REG, HEATER_PIN_BIT);
    } else {
        REG_WRITE(GPIO_OUT1_W1TC_REG, HEATER_PIN_BIT);
    }
}

static void task_heater(void *) {
    uint32_t tick = 0;

    float slope_hist[IMU_HEATER_SLOPE_WINDOW] = {};
    int   slope_idx = 0;
    int   slope_cnt = 0;
    float prev_temp = 0.0f;
    bool  prev_valid = false;

    while (1) {
        taskENTER_CRITICAL(&g_state_mux);
        float temp = g_state.temp;
        taskEXIT_CRITICAL(&g_state_mux);

        // Phase 1 — preheat: full-on until within PREHEAT_MARGIN of target
        // Phase 2 — regulate: bang-bang once near target
        bool near_target = (temp >= s_target - IMU_HEATER_PREHEAT_MARGIN);
        if (!near_target) {
            heater_set(true);
        } else {
            if (s_on  && temp >= s_target + HEATER_HYSTERESIS) heater_set(false);
            if (!s_on && temp <  s_target - HEATER_HYSTERESIS) heater_set(true);
        }

        // Milestone 1 — 38°C: warm enough to use
        if (!s_warm && temp >= IMU_HEATER_WARM_C) {
            s_warm = true;
        }

        // Milestone 2 — 54°C: thermal steady state
        if (!s_stable && temp >= IMU_HEATER_STABLE_C) {
            s_stable = true;
        }

        // Slope tracking: accumulate per-sample delta once near target
        if (!s_ready && near_target) {
            if (prev_valid) {
                float delta = temp - prev_temp;
                slope_hist[slope_idx] = delta;
                slope_idx = (slope_idx + 1) % IMU_HEATER_SLOPE_WINDOW;
                if (slope_cnt < IMU_HEATER_SLOPE_WINDOW) slope_cnt++;
            }
            prev_temp  = temp;
            prev_valid = true;
        }

        // Ready condition: in target band + slope window full + max |delta| < threshold
        if (!s_ready) {
            bool in_band   = (temp >= s_target - HEATER_HYSTERESIS);
            bool timed_out = s_stable &&
                ((tick * 500) >= IMU_HEATER_READY_TIMEOUT);
            bool slope_ok  = false;

            if (slope_cnt == IMU_HEATER_SLOPE_WINDOW) {
                slope_ok = true;
                for (int i = 0; i < IMU_HEATER_SLOPE_WINDOW; i++) {
                    if (fabsf(slope_hist[i]) >= IMU_HEATER_SLOPE_MAX) {
                        slope_ok = false;
                        break;
                    }
                }
            }

            if ((in_band && slope_ok) || timed_out) {
                s_ready = true;
            }
        }

        tick++;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void imu_heater_init(void) {
    // GPIO46 is a strapping pin — configure output directly via registers
    PIN_FUNC_SELECT(IO_MUX_GPIO46_REG, FUNC_GPIO46_GPIO46);
    REG_WRITE(GPIO_ENABLE1_W1TS_REG, HEATER_PIN_BIT);

    heater_set(false);

    xTaskCreate(task_heater, "heater", 2048, NULL, 2, NULL);
}

void imu_heater_set_target(float temp_c) { s_target = temp_c; }
float imu_heater_get_target(void)        { return s_target; }
bool  imu_heater_is_on(void)             { return s_on; }
bool  imu_heater_ready(void)             { return s_ready; }
bool  imu_heater_warm(void)              { return s_warm; }
bool  imu_heater_stable(void)            { return s_stable; }
