#include "nvs_params.h"
#include "config.h"
#include "fw_log.h"
#include "steering_utils.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <string.h>

typedef enum {
    NVS_JOB_PID,
    NVS_JOB_MAG,
    NVS_JOB_CHASSIS,
    NVS_JOB_SPEED,
    NVS_JOB_LEVEL_RESET,
} nvs_job_type_t;

typedef struct {
    nvs_job_type_t type;
    float a;
    float b;
    float c;
    float mag_hard[3];
    float mag_soft[9];
    int value;
} nvs_job_t;

static QueueHandle_t s_nvs_queue = NULL;

static bool nvs_get_blob_exact(nvs_handle_t h, const char *key, void *out, size_t expected) {
    size_t size = expected;
    return nvs_get_blob(h, key, out, &size) == ESP_OK && size == expected;
}

static void nvs_load_pid_defaults(float *kp, float *ki, float *kd) {
    *kp = PID_KP_DEFAULT;
    *ki = PID_KI_DEFAULT;
    *kd = PID_KD_DEFAULT;
}

static void nvs_load_mag_defaults(float hard[3], float soft[9]);

static void nvs_load_chassis_defaults(float *odom_scale, float *steering_trim_deg) {
    *odom_scale = ODOM_SCALE_DEFAULT;
    *steering_trim_deg = STEERING_TRIM_DEFAULT_DEG;
}

static void nvs_load_speed_defaults(int *deadband_us) {
    *deadband_us = THROTTLE_FF_DEADBAND_DEFAULT_US;
}

static void nvs_worker(void *) {
    nvs_job_t job;
    while (xQueueReceive(s_nvs_queue, &job, portMAX_DELAY) == pdTRUE) {
        switch (job.type) {
            case NVS_JOB_PID:
                nvs_params_save(job.a, job.b, job.c);
                break;
            case NVS_JOB_MAG:
                nvs_mag_save(job.mag_hard, job.mag_soft);
                break;
            case NVS_JOB_CHASSIS:
                nvs_chassis_save(job.a, job.b);
                break;
            case NVS_JOB_SPEED:
                nvs_speed_deadband_save(job.value);
                break;
            case NVS_JOB_LEVEL_RESET:
                nvs_level_reset();
                break;
        }
    }
}

static bool nvs_enqueue(const nvs_job_t *job) {
    return s_nvs_queue && xQueueSend(s_nvs_queue, job, 0) == pdTRUE;
}

void nvs_params_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    if (!s_nvs_queue) {
        s_nvs_queue = xQueueCreate(8, sizeof(nvs_job_t));
        if (!s_nvs_queue ||
            xTaskCreate(nvs_worker, "nvs_save", 4096, NULL, 1, NULL) != pdPASS) {
            FW_LOGF("WARN: NVS async worker init failed\n");
            s_nvs_queue = NULL;
        }
    }
}

void nvs_params_load(float *kp, float *ki, float *kd) {
    nvs_handle_t h;
    if (nvs_open(PREF_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        *kp = PID_KP_DEFAULT; *ki = PID_KI_DEFAULT; *kd = PID_KD_DEFAULT;
        nvs_params_save(PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);
        return;
    }

    uint8_t initialized = 0;
    if (nvs_get_u8(h, PREF_KEY_INIT, &initialized) != ESP_OK) {
        initialized = 0;
    }

    if (initialized) {
        uint32_t profile = 0;
        if (nvs_get_u32(h, PREF_KEY_PID_PROFILE, &profile) != ESP_OK ||
            profile != PID_PROFILE_VERSION) {
            nvs_load_pid_defaults(kp, ki, kd);
            nvs_close(h);
            nvs_params_save(PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);
            FW_LOGF("INFO: PID profile updated to factory defaults\n");
            return;
        }

        bool ok =
            nvs_get_blob_exact(h, PREF_KEY_KP, kp, sizeof(float)) &&
            nvs_get_blob_exact(h, PREF_KEY_KI, ki, sizeof(float)) &&
            nvs_get_blob_exact(h, PREF_KEY_KD, kd, sizeof(float));
        if (!ok) {
            nvs_load_pid_defaults(kp, ki, kd);
            FW_LOGF("WARN: invalid PID NVS, using defaults\n");
        }
    } else {
        nvs_load_pid_defaults(kp, ki, kd);
        nvs_close(h);
        nvs_params_save(PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);
        return;
    }
    nvs_close(h);
}

bool nvs_params_save(float kp, float ki, float kd) {
    nvs_handle_t h;
    if (nvs_open(PREF_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return false;
    uint8_t flag = 1;
    bool ok =
        nvs_set_blob(h, PREF_KEY_KP, &kp, sizeof(float)) == ESP_OK &&
        nvs_set_blob(h, PREF_KEY_KI, &ki, sizeof(float)) == ESP_OK &&
        nvs_set_blob(h, PREF_KEY_KD, &kd, sizeof(float)) == ESP_OK &&
        nvs_set_u8(h, PREF_KEY_INIT, flag) == ESP_OK &&
        nvs_set_u32(h, PREF_KEY_PID_PROFILE, PID_PROFILE_VERSION) == ESP_OK;
    if (ok) ok = nvs_commit(h) == ESP_OK;
    if (!ok) FW_LOGF("WARN: PID NVS save failed\n");
    nvs_close(h);
    return ok;
}

bool nvs_params_save_async(float kp, float ki, float kd) {
    nvs_job_t job = {};
    job.type = NVS_JOB_PID;
    job.a = kp;
    job.b = ki;
    job.c = kd;
    return nvs_enqueue(&job);
}

static const float k_identity_soft[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f,
};

bool nvs_mag_save(const float hard[3], const float soft[9]) {
    nvs_handle_t h;
    if (nvs_open(PREF_NS_MAG, NVS_READWRITE, &h) != ESP_OK) return false;
    uint8_t flag = 1;
    bool ok =
        nvs_set_blob(h, PREF_KEY_MAG_HI, hard, 3 * sizeof(float)) == ESP_OK &&
        nvs_set_blob(h, PREF_KEY_MAG_SI, soft, 9 * sizeof(float)) == ESP_OK &&
        nvs_set_u8(h, PREF_KEY_MAG_INIT, flag) == ESP_OK;
    if (ok) ok = nvs_commit(h) == ESP_OK;
    if (!ok) FW_LOGF("WARN: mag NVS save failed\n");
    nvs_close(h);
    return ok;
}

bool nvs_mag_save_async(const float hard[3], const float soft[9]) {
    nvs_job_t job = {};
    job.type = NVS_JOB_MAG;
    memcpy(job.mag_hard, hard, sizeof(job.mag_hard));
    memcpy(job.mag_soft, soft, sizeof(job.mag_soft));
    return nvs_enqueue(&job);
}

bool nvs_cf_save(float alpha_static, float alpha_moving, float speed_thr) {
    nvs_handle_t h;
    if (nvs_open(PREF_NS_CF, NVS_READWRITE, &h) != ESP_OK) return false;
    uint8_t flag = 1;
    bool ok =
        nvs_set_blob(h, PREF_KEY_CF_AS,  &alpha_static,  sizeof(float)) == ESP_OK &&
        nvs_set_blob(h, PREF_KEY_CF_AM,  &alpha_moving,  sizeof(float)) == ESP_OK &&
        nvs_set_blob(h, PREF_KEY_CF_THR, &speed_thr,     sizeof(float)) == ESP_OK &&
        nvs_set_u8(h, PREF_KEY_CF_INIT, flag) == ESP_OK;
    if (ok) ok = nvs_commit(h) == ESP_OK;
    if (!ok) FW_LOGF("WARN: CF NVS save failed\n");
    nvs_close(h);
    return ok;
}

void nvs_cf_load(float *alpha_static, float *alpha_moving, float *speed_thr) {
    *alpha_static = COMPASS_CF_ALPHA_STATIC;
    *alpha_moving = COMPASS_CF_ALPHA_MOVING;
    *speed_thr    = COMPASS_CF_SPEED_THR;

    nvs_handle_t h;
    if (nvs_open(PREF_NS_CF, NVS_READONLY, &h) != ESP_OK) return;

    uint8_t initialized = 0;
    if (nvs_get_u8(h, PREF_KEY_CF_INIT, &initialized) != ESP_OK) {
        initialized = 0;
    }
    if (initialized) {
        bool ok =
            nvs_get_blob_exact(h, PREF_KEY_CF_AS,  alpha_static, sizeof(float)) &&
            nvs_get_blob_exact(h, PREF_KEY_CF_AM,  alpha_moving, sizeof(float)) &&
            nvs_get_blob_exact(h, PREF_KEY_CF_THR, speed_thr,    sizeof(float));
        if (!ok) {
            *alpha_static = COMPASS_CF_ALPHA_STATIC;
            *alpha_moving = COMPASS_CF_ALPHA_MOVING;
            *speed_thr    = COMPASS_CF_SPEED_THR;
            FW_LOGF("WARN: invalid CF NVS, using defaults\n");
        }
    }
    nvs_close(h);
}

bool nvs_level_save(float ox, float oy, float oz) {
    nvs_handle_t h;
    if (nvs_open(PREF_NS_LEVEL, NVS_READWRITE, &h) != ESP_OK) return false;
    uint8_t flag = 1;
    bool ok =
        nvs_set_blob(h, PREF_KEY_LEVEL_OX, &ox, sizeof(float)) == ESP_OK &&
        nvs_set_blob(h, PREF_KEY_LEVEL_OY, &oy, sizeof(float)) == ESP_OK &&
        nvs_set_blob(h, PREF_KEY_LEVEL_OZ, &oz, sizeof(float)) == ESP_OK &&
        nvs_set_u8(h, PREF_KEY_LEVEL_INIT, flag) == ESP_OK;
    if (ok) ok = nvs_commit(h) == ESP_OK;
    if (!ok) FW_LOGF("WARN: level NVS save failed\n");
    nvs_close(h);
    return ok;
}

bool nvs_level_load(float *ox, float *oy, float *oz) {
    *ox = 0.0f; *oy = 0.0f; *oz = 0.0f;
    nvs_handle_t h;
    if (nvs_open(PREF_NS_LEVEL, NVS_READONLY, &h) != ESP_OK) return false;
    uint8_t initialized = 0;
    if (nvs_get_u8(h, PREF_KEY_LEVEL_INIT, &initialized) != ESP_OK) {
        initialized = 0;
    }
    if (initialized) {
        bool ok =
            nvs_get_blob_exact(h, PREF_KEY_LEVEL_OX, ox, sizeof(float)) &&
            nvs_get_blob_exact(h, PREF_KEY_LEVEL_OY, oy, sizeof(float)) &&
            nvs_get_blob_exact(h, PREF_KEY_LEVEL_OZ, oz, sizeof(float));
        if (!ok) {
            *ox = 0.0f; *oy = 0.0f; *oz = 0.0f;
            initialized = 0;
            FW_LOGF("WARN: invalid level NVS, using zero offset\n");
        }
    }
    nvs_close(h);
    return initialized != 0;
}

bool nvs_level_reset(void) {
    nvs_handle_t h;
    if (nvs_open(PREF_NS_LEVEL, NVS_READWRITE, &h) != ESP_OK) return false;
    bool ok = nvs_erase_all(h) == ESP_OK;
    if (ok) ok = nvs_commit(h) == ESP_OK;
    if (!ok) FW_LOGF("WARN: level NVS reset failed\n");
    nvs_close(h);
    return ok;
}

bool nvs_level_reset_async(void) {
    nvs_job_t job = {};
    job.type = NVS_JOB_LEVEL_RESET;
    return nvs_enqueue(&job);
}

static void nvs_load_mag_defaults(float hard[3], float soft[9]) {
    memset(hard, 0, 3 * sizeof(float));
    memcpy(soft, k_identity_soft, 9 * sizeof(float));
}

bool nvs_mag_load(float hard[3], float soft[9]) {
    nvs_handle_t h;
    if (nvs_open(PREF_NS_MAG, NVS_READONLY, &h) != ESP_OK) {
        nvs_load_mag_defaults(hard, soft);
        return false;
    }

    uint8_t initialized = 0;
    if (nvs_get_u8(h, PREF_KEY_MAG_INIT, &initialized) != ESP_OK) {
        initialized = 0;
    }

    if (initialized) {
        bool ok =
            nvs_get_blob_exact(h, PREF_KEY_MAG_HI, hard, 3 * sizeof(float)) &&
            nvs_get_blob_exact(h, PREF_KEY_MAG_SI, soft, 9 * sizeof(float));
        if (!ok) {
            nvs_load_mag_defaults(hard, soft);
            FW_LOGF("WARN: invalid mag NVS, using identity calibration\n");
            initialized = 0;
        }
    } else {
        nvs_load_mag_defaults(hard, soft);
    }
    nvs_close(h);
    return initialized != 0;
}

bool nvs_chassis_save(float odom_scale, float steering_trim_deg) {
    nvs_handle_t h;
    if (nvs_open(PREF_NS_CHASSIS, NVS_READWRITE, &h) != ESP_OK) return false;
    uint8_t flag = 1;
    bool ok =
        nvs_set_blob(h, PREF_KEY_ODOM_SCALE, &odom_scale, sizeof(float)) == ESP_OK &&
        nvs_set_blob(h, PREF_KEY_STEERING_TRIM_DEG, &steering_trim_deg, sizeof(float)) == ESP_OK &&
        nvs_set_u8(h, PREF_KEY_CHASSIS_INIT, flag) == ESP_OK;
    if (ok) ok = nvs_commit(h) == ESP_OK;
    if (!ok) FW_LOGF("WARN: chassis NVS save failed\n");
    nvs_close(h);
    return ok;
}

bool nvs_chassis_save_async(float odom_scale, float steering_trim_deg) {
    nvs_job_t job = {};
    job.type = NVS_JOB_CHASSIS;
    job.a = odom_scale;
    job.b = steering_trim_deg;
    return nvs_enqueue(&job);
}

void nvs_chassis_load(float *odom_scale, float *steering_trim_deg) {
    nvs_load_chassis_defaults(odom_scale, steering_trim_deg);

    nvs_handle_t h;
    if (nvs_open(PREF_NS_CHASSIS, NVS_READONLY, &h) != ESP_OK) return;

    uint8_t initialized = 0;
    if (nvs_get_u8(h, PREF_KEY_CHASSIS_INIT, &initialized) != ESP_OK) {
        initialized = 0;
    }

    if (initialized) {
        float trim_deg = STEERING_TRIM_DEFAULT_DEG;
        int32_t legacy_trim_us = 0;
        bool has_trim_deg =
            nvs_get_blob_exact(h, PREF_KEY_STEERING_TRIM_DEG, &trim_deg, sizeof(float));
        bool has_legacy_trim =
            nvs_get_i32(h, PREF_KEY_STEERING_TRIM_US, &legacy_trim_us) == ESP_OK;
        bool ok =
            nvs_get_blob_exact(h, PREF_KEY_ODOM_SCALE, odom_scale, sizeof(float)) &&
            (has_trim_deg || has_legacy_trim);
        if (ok) {
            *odom_scale = CONSTRAIN(*odom_scale, ODOM_SCALE_MIN, ODOM_SCALE_MAX);
            if (!has_trim_deg && has_legacy_trim) {
                if (legacy_trim_us < STEERING_TRIM_LEGACY_MIN_US) {
                    legacy_trim_us = STEERING_TRIM_LEGACY_MIN_US;
                }
                if (legacy_trim_us > STEERING_TRIM_LEGACY_MAX_US) {
                    legacy_trim_us = STEERING_TRIM_LEGACY_MAX_US;
                }
                trim_deg = steering_trim_us_to_deg((int)legacy_trim_us);
            }
            *steering_trim_deg = CONSTRAIN(trim_deg, STEERING_TRIM_MIN_DEG, STEERING_TRIM_MAX_DEG);
        } else {
            nvs_load_chassis_defaults(odom_scale, steering_trim_deg);
            FW_LOGF("WARN: invalid chassis NVS, using defaults\n");
        }
    }

    nvs_close(h);
}

bool nvs_speed_deadband_save(int deadband_us) {
    nvs_handle_t h;
    if (nvs_open(PREF_NS_SPEED, NVS_READWRITE, &h) != ESP_OK) return false;
    uint8_t flag = 1;
    deadband_us = CONSTRAIN(deadband_us, THROTTLE_FF_DEADBAND_MIN_US, THROTTLE_FF_DEADBAND_MAX_US);
    bool ok =
        nvs_set_i32(h, PREF_KEY_SPEED_DEADBAND, deadband_us) == ESP_OK &&
        nvs_set_u8(h, PREF_KEY_SPEED_INIT, flag) == ESP_OK;
    if (ok) ok = nvs_commit(h) == ESP_OK;
    if (!ok) FW_LOGF("WARN: speed NVS save failed\n");
    nvs_close(h);
    return ok;
}

bool nvs_speed_deadband_save_async(int deadband_us) {
    nvs_job_t job = {};
    job.type = NVS_JOB_SPEED;
    job.value = deadband_us;
    return nvs_enqueue(&job);
}

void nvs_speed_deadband_load(int *deadband_us) {
    nvs_load_speed_defaults(deadband_us);

    nvs_handle_t h;
    if (nvs_open(PREF_NS_SPEED, NVS_READONLY, &h) != ESP_OK) return;

    uint8_t initialized = 0;
    if (nvs_get_u8(h, PREF_KEY_SPEED_INIT, &initialized) == ESP_OK && initialized) {
        int32_t saved = THROTTLE_FF_DEADBAND_DEFAULT_US;
        if (nvs_get_i32(h, PREF_KEY_SPEED_DEADBAND, &saved) == ESP_OK) {
            *deadband_us = CONSTRAIN(saved, THROTTLE_FF_DEADBAND_MIN_US, THROTTLE_FF_DEADBAND_MAX_US);
        } else {
            FW_LOGF("WARN: invalid speed NVS, using defaults\n");
        }
    }

    nvs_close(h);
}
