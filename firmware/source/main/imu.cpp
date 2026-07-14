#include "imu.h"
#include "config.h"
#include "fw_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

#define QMI_REG_WHO_AM_I    0x00
#define QMI_REG_CTRL1       0x02
#define QMI_REG_CTRL2       0x03
#define QMI_REG_CTRL3       0x04
#define QMI_REG_CTRL7       0x08
#define QMI_REG_STATUS1     0x2E
#define QMI_REG_TEMP_L      0x33
#define QMI_REG_AX_L        0x35

#define QMI_WHO_AM_I_VAL    0x05

#define HISTORY_LEN      10
#define IMU_AVG_SAMPLES  5
#define ACCEL_SCALE      8192.0f   // LSB/g  for ±4g  (32768/4)
#define GYRO_SCALE       32.0f     // LSB/dps for ±1024dps (32768/1024)

// Bias sanity: reject per-update step > 0.5 dps (glitch guard)
#define BIAS_STEP_MAX       0.5f

static i2c_master_dev_handle_t s_dev = NULL;

static float s_acc_hist[HISTORY_LEN][3];
static float s_gyr_hist[HISTORY_LEN][3];
static int   s_hist_idx   = 0;
static int   s_hist_count = 0;

static float s_bias[3]        = {0};
static float s_accel_offset[3] = {0};  // g, mounting level offsets (ox, oy, oz=az_mean-1)
static bool  s_bias_ready   = false;
static bool  s_is_static    = false;
static int   s_static_count = 0;
static float s_wheel_speed  = 0.0f;

static float s_batch_sum[3] = {0};
static int   s_batch_count  = 0;
static float s_static_residual_gnorm = 0.0f;

static float s_avg_acc[3] = {0};
static float s_avg_gyr[3] = {0};
static float s_avg_temp   = 0.0f;
static int   s_avg_n      = 0;

static void write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_master_transmit(s_dev, buf, 2, 50);
}

static uint8_t read_reg(uint8_t reg) {
    uint8_t val = 0;
    i2c_master_transmit_receive(s_dev, &reg, 1, &val, 1, 50);
    return val;
}

static bool read_burst(uint8_t reg, uint8_t *buf, uint8_t len) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, 50) == ESP_OK;
}

void imu_init(i2c_master_dev_handle_t dev) {
    s_dev = dev;
    memset(s_acc_hist, 0, sizeof(s_acc_hist));
    memset(s_gyr_hist, 0, sizeof(s_gyr_hist));

#if FW_VERBOSE_LOG
    uint8_t whoami = read_reg(QMI_REG_WHO_AM_I);
    FW_LOGF("INFO: QMI8658 WHO_AM_I=0x%02X%s\n",
            whoami,
            whoami == QMI_WHO_AM_I_VAL ? "" : " (unexpected)");
#else
    (void)read_reg(QMI_REG_WHO_AM_I);
#endif
    write_reg(QMI_REG_CTRL1, 0x40);  // address auto-increment, little-endian
    write_reg(QMI_REG_CTRL2, 0x13);  // accel ±4g, 1000Hz ODR
    write_reg(QMI_REG_CTRL3, 0x63);  // gyro ±1024dps, 1000Hz ODR
    write_reg(QMI_REG_CTRL7, 0x03);  // enable accel + gyro

    vTaskDelay(pdMS_TO_TICKS(50));
}

void imu_calibrate_bias(void) {
    float sum[3] = {0};
    float sumsq[3] = {0};
    int count = 0;
    int attempts = 0;
    while (count < GYRO_BIAS_SAMPLES && attempts < GYRO_BIAS_SAMPLES * 4) {
        attempts++;
        uint8_t status = read_reg(QMI_REG_STATUS1);
        if (!(status & 0x03)) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }
        uint8_t raw[14];
        if (!read_burst(QMI_REG_TEMP_L, raw, 14)) continue;
        int16_t gx = (int16_t)((raw[9]  << 8) | raw[8]);
        int16_t gy = (int16_t)((raw[11] << 8) | raw[10]);
        int16_t gz = (int16_t)((raw[13] << 8) | raw[12]);
        float gxd = gx / GYRO_SCALE;
        float gyd = gy / GYRO_SCALE;
        float gzd = gz / GYRO_SCALE;
        sum[0] += gxd;
        sum[1] += gyd;
        sum[2] += gzd;
        sumsq[0] += gxd * gxd;
        sumsq[1] += gyd * gyd;
        sumsq[2] += gzd * gzd;
        count++;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (count >= GYRO_BIAS_SAMPLES) {
        float nb[3] = { sum[0]/count, sum[1]/count, sum[2]/count };
        float std[3] = {0};
        for (int i = 0; i < 3; i++) {
            float var = sumsq[i] / count - nb[i] * nb[i];
            std[i] = sqrtf(fmaxf(var, 0.0f));
        }

        if (fabsf(nb[0]) < GYRO_BIAS_INIT_ABS_MAX
         && fabsf(nb[1]) < GYRO_BIAS_INIT_ABS_MAX
         && fabsf(nb[2]) < GYRO_BIAS_INIT_ABS_MAX
         && std[0] < GYRO_BIAS_INIT_STD_MAX
         && std[1] < GYRO_BIAS_INIT_STD_MAX
         && std[2] < GYRO_BIAS_INIT_STD_MAX) {
            s_bias[0] = nb[0];
            s_bias[1] = nb[1];
            s_bias[2] = nb[2];
            s_bias_ready = true;
        }
    }
}

static bool detect_static(void) {
    if (s_hist_count < HISTORY_LEN) return false;

    if (fabsf(s_wheel_speed) > STATIC_WHEEL_SPEED_MAX) return false;

    float am[3] = {0};
    float gm[3] = {0};
    for (int i = 0; i < HISTORY_LEN; i++) {
        am[0] += s_acc_hist[i][0];
        am[1] += s_acc_hist[i][1];
        am[2] += s_acc_hist[i][2];
        gm[0] += s_gyr_hist[i][0];
        gm[1] += s_gyr_hist[i][1];
        gm[2] += s_gyr_hist[i][2];
    }
    for (int j = 0; j < 3; j++) {
        am[j] /= HISTORY_LEN;
        gm[j] /= HISTORY_LEN;
    }

    float acc_var = 0.0f;
    float gyr_var = 0.0f;
    for (int i = 0; i < HISTORY_LEN; i++) {
        float dx = s_acc_hist[i][0] - am[0];
        float dy = s_acc_hist[i][1] - am[1];
        float dz = s_acc_hist[i][2] - am[2];
        acc_var += dx * dx + dy * dy + dz * dz;

        float dgx = s_gyr_hist[i][0] - gm[0];
        float dgy = s_gyr_hist[i][1] - gm[1];
        float dgz = s_gyr_hist[i][2] - gm[2];
        gyr_var += dgx * dgx + dgy * dgy + dgz * dgz;
    }
    acc_var /= (float)(HISTORY_LEN * 3);
    gyr_var /= (float)(HISTORY_LEN * 3);

    float amag = sqrtf(am[0] * am[0] + am[1] * am[1] + am[2] * am[2]);
    if (fabsf(amag - 1.0f) > STATIC_AMAG_TOL) return false;
    if (acc_var > STATIC_ACC_STD_MAX * STATIC_ACC_STD_MAX) return false;

    float raw_gnorm = sqrtf(gm[0]*gm[0] + gm[1]*gm[1] + gm[2]*gm[2]);
    float gstd = sqrtf(gyr_var);
    if (gstd > STATIC_GYR_STD_MAX) return false;

    if (!s_bias_ready) {
        return raw_gnorm < STATIC_GYR_INIT_NORM_MAX;
    }

    float bgx = gm[0] - s_bias[0];
    float bgy = gm[1] - s_bias[1];
    float bgz = gm[2] - s_bias[2];
    s_static_residual_gnorm = sqrtf(bgx*bgx + bgy*bgy + bgz*bgz);
    return s_static_residual_gnorm < STATIC_GYR_NORM_MAX;
}

static void update_bias(float gx, float gy, float gz) {
    bool currently_static = detect_static();

    if (currently_static) {
        s_static_count++;
        if (s_static_count > STATIONARY_COUNT_THRESHOLD) {
            if (!s_is_static) {
                s_is_static  = true;
            }

            s_batch_sum[0] += gx;
            s_batch_sum[1] += gy;
            s_batch_sum[2] += gz;
            s_batch_count++;

            if (s_batch_count >= GYRO_BIAS_BATCH_SAMPLES) {
                float nb[3] = {
                    s_batch_sum[0] / s_batch_count,
                    s_batch_sum[1] / s_batch_count,
                    s_batch_sum[2] / s_batch_count,
                };
                // Reject if batch mean deviates > 5 dps from current bias (sensor fault guard)
                float db[3] = {
                    nb[0] - s_bias[0],
                    nb[1] - s_bias[1],
                    nb[2] - s_bias[2],
                };
                float db_norm = sqrtf(db[0]*db[0] + db[1]*db[1] + db[2]*db[2]);
                if (s_bias_ready
                 && fabsf(db[0]) < BIAS_STEP_MAX * 10.0f
                 && fabsf(db[1]) < BIAS_STEP_MAX * 10.0f
                 && fabsf(db[2]) < BIAS_STEP_MAX * 10.0f
                 && db_norm >= GYRO_BIAS_UPDATE_THRESHOLD) {
                    float scale = 1.0f;
                    if (db_norm > GYRO_BIAS_UPDATE_GYR_MAX) {
                        scale = GYRO_BIAS_UPDATE_GYR_MAX / db_norm;
                    }
                    s_bias[0] += GYRO_BIAS_UPDATE_ALPHA * db[0] * scale;
                    s_bias[1] += GYRO_BIAS_UPDATE_ALPHA * db[1] * scale;
                    s_bias[2] += GYRO_BIAS_UPDATE_ALPHA * db[2] * scale;
                }
                s_batch_sum[0] = s_batch_sum[1] = s_batch_sum[2] = 0.0f;
                s_batch_count  = 0;
            }
        }
    } else {
        s_static_count = 0;
        s_is_static    = false;
        s_batch_sum[0] = s_batch_sum[1] = s_batch_sum[2] = 0.0f;
        s_batch_count  = 0;
    }
}

bool imu_update(imu_data_t *out) {
    uint8_t status = read_reg(QMI_REG_STATUS1);
    if (!(status & 0x03)) return false;

    uint8_t raw[14];
    if (!read_burst(QMI_REG_TEMP_L, raw, 14)) return false;

    int16_t temp_raw = (int16_t)((raw[1] << 8) | raw[0]);
    s_avg_temp += temp_raw / 256.0f;

    int16_t ax = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t ay = (int16_t)((raw[5] << 8) | raw[4]);
    int16_t az = (int16_t)((raw[7] << 8) | raw[6]);
    s_avg_acc[0] += ax / ACCEL_SCALE;
    s_avg_acc[1] += ay / ACCEL_SCALE;
    s_avg_acc[2] += az / ACCEL_SCALE;

    int16_t gx = (int16_t)((raw[9]  << 8) | raw[8]);
    int16_t gy = (int16_t)((raw[11] << 8) | raw[10]);
    int16_t gz = (int16_t)((raw[13] << 8) | raw[12]);
    s_avg_gyr[0] += gx / GYRO_SCALE;
    s_avg_gyr[1] += gy / GYRO_SCALE;
    s_avg_gyr[2] += gz / GYRO_SCALE;

    if (++s_avg_n < IMU_AVG_SAMPLES) return false;

    out->temp = s_avg_temp / IMU_AVG_SAMPLES;
    float raw_acc[3] = {
        s_avg_acc[0] / IMU_AVG_SAMPLES,
        s_avg_acc[1] / IMU_AVG_SAMPLES,
        s_avg_acc[2] / IMU_AVG_SAMPLES,
    };
    out->accelX = raw_acc[0] - s_accel_offset[0];
    out->accelY = raw_acc[1] - s_accel_offset[1];
    out->accelZ = raw_acc[2] - s_accel_offset[2];

    float gxf = s_avg_gyr[0] / IMU_AVG_SAMPLES;
    float gyf = s_avg_gyr[1] / IMU_AVG_SAMPLES;
    float gzf = s_avg_gyr[2] / IMU_AVG_SAMPLES;

    s_avg_acc[0] = s_avg_acc[1] = s_avg_acc[2] = 0.0f;
    s_avg_gyr[0] = s_avg_gyr[1] = s_avg_gyr[2] = 0.0f;
    s_avg_temp = 0.0f;
    s_avg_n    = 0;

    // Static detection must not depend on saved level offsets; a bad offset
    // would otherwise block the stillness gate and prevent recalibration.
    s_acc_hist[s_hist_idx][0] = raw_acc[0];
    s_acc_hist[s_hist_idx][1] = raw_acc[1];
    s_acc_hist[s_hist_idx][2] = raw_acc[2];
    s_gyr_hist[s_hist_idx][0] = gxf;
    s_gyr_hist[s_hist_idx][1] = gyf;
    s_gyr_hist[s_hist_idx][2] = gzf;
    s_hist_idx = (s_hist_idx + 1) % HISTORY_LEN;
    if (s_hist_count < HISTORY_LEN) s_hist_count++;

    update_bias(gxf, gyf, gzf);

    if (s_bias_ready) {
        gxf -= s_bias[0];
        gyf -= s_bias[1];
        gzf -= s_bias[2];
    }

    out->gyroX = gxf;
    out->gyroY = gyf;
    out->gyroZ = gzf;

    return true;
}

bool imu_is_static(void)  { return s_is_static; }
bool imu_bias_ready(void) { return s_bias_ready; }

void imu_get_bias(float *bx, float *by, float *bz) {
    *bx = s_bias[0];
    *by = s_bias[1];
    *bz = s_bias[2];
}

void imu_set_level_offset(float ox, float oy, float oz) {
    s_accel_offset[0] = ox;
    s_accel_offset[1] = oy;
    s_accel_offset[2] = oz;
}

void imu_get_level_offset(float *ox, float *oy, float *oz) {
    *ox = s_accel_offset[0];
    *oy = s_accel_offset[1];
    *oz = s_accel_offset[2];
}

void imu_set_wheel_speed(float speed_mps) {
    s_wheel_speed = speed_mps;
}
