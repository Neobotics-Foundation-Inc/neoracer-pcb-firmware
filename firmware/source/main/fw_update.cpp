#include "fw_update.h"

#include "app_state.h"
#include "config.h"
#include "pid.h"
#include "serial_tx.h"
#include "steering_utils.h"
#include "version.h"

#include "esp_ota_ops.h"
#include "esp_idf_version.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/sha.h"

#include <ctype.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

extern pid_ctrl_t g_pid;
extern portMUX_TYPE g_state_mux;

static bool s_active = false;
static esp_ota_handle_t s_handle = 0;
static const esp_partition_t *s_partition = NULL;
static uint32_t s_expected_size = 0;
static uint32_t s_written = 0;
static uint32_t s_next_seq = 0;
static uint8_t s_expected_sha[32] = {0};
static SHA_CTX s_sha_ctx;

static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static bool parse_sha256(const char *hex, uint8_t out[32]) {
    if (!hex || strlen(hex) != 64) return false;
    for (int i = 0; i < 32; i++) {
        int hi = hex_nibble(hex[i * 2]);
        int lo = hex_nibble(hex[i * 2 + 1]);
        if (hi < 0 || lo < 0) return false;
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return true;
}

static bool decode_hex_payload(const char *hex, uint8_t *out, size_t out_cap, size_t *out_len) {
    size_t len = strlen(hex);
    if ((len % 2) != 0 || (len / 2) > out_cap) return false;

    for (size_t i = 0; i < len / 2; i++) {
        int hi = hex_nibble(hex[i * 2]);
        int lo = hex_nibble(hex[i * 2 + 1]);
        if (hi < 0 || lo < 0) return false;
        out[i] = (uint8_t)((hi << 4) | lo);
    }

    *out_len = len / 2;
    return true;
}

static void force_safe_stop(void) {
    taskENTER_CRITICAL(&g_state_mux);
    g_state.target_speed = 0.0f;
    g_state.steering_pulse = steering_trimmed_pulse(STEERING_CENTER, g_state.steering_trim_deg);
    g_state.remote_active = false;
    g_state.serial_control_active = true;
    g_state.last_serial_cmd_ms = (unsigned long)(esp_timer_get_time() / 1000);
    taskEXIT_CRITICAL(&g_state_mux);

    pid_reset(&g_pid);
}

static void reset_state(void) {
    s_active = false;
    s_handle = 0;
    s_partition = NULL;
    s_expected_size = 0;
    s_written = 0;
    s_next_seq = 0;
    memset(s_expected_sha, 0, sizeof(s_expected_sha));
}

static void abort_update(void) {
    if (s_active && s_handle != 0) {
        esp_ota_abort(s_handle);
    }
    reset_state();
}

bool fw_update_active(void) {
    return s_active;
}

void fw_update_mark_app_valid(void) {
    esp_ota_mark_app_valid_cancel_rollback();
}

static void handle_begin(const char *args) {
    unsigned long size_ul = 0;
    char sha_hex[65] = {0};
    char option[16] = {0};
    if (sscanf(args, "%lu %64s", &size_ul, sha_hex) != 2) {
        serial_tx_printf("ERROR fw begin usage: fw begin <size> <sha256>\n");
        return;
    }
    (void)sscanf(args, "%lu %64s %15s", &size_ul, sha_hex, option);
    bool force_low_voltage = strcmp(option, "force") == 0;

    if (s_active) {
        serial_tx_printf("ERROR fw busy\n");
        return;
    }

    force_safe_stop();

    float voltage = 0.0f;
    taskENTER_CRITICAL(&g_state_mux);
    voltage = g_state.voltage;
    taskEXIT_CRITICAL(&g_state_mux);
    if (!force_low_voltage && voltage > 1.0f && voltage < LOW_VOLTAGE_RECOVER) {
        serial_tx_printf("ERROR fw low_voltage %.1fV\n", voltage);
        return;
    }

    if (size_ul == 0 || size_ul > UINT32_MAX) {
        serial_tx_printf("ERROR fw bad_size\n");
        return;
    }

    if (!parse_sha256(sha_hex, s_expected_sha)) {
        serial_tx_printf("ERROR fw bad_sha256\n");
        return;
    }

    s_partition = esp_ota_get_next_update_partition(NULL);
    if (!s_partition) {
        serial_tx_printf("ERROR fw no_ota_partition\n");
        reset_state();
        return;
    }

    s_expected_size = (uint32_t)size_ul;
    if (s_expected_size > s_partition->size) {
        serial_tx_printf("ERROR fw too_large size=%" PRIu32 " max=%" PRIu32 "\n",
                         s_expected_size,
                         (uint32_t)s_partition->size);
        reset_state();
        return;
    }

    esp_err_t err = esp_ota_begin(s_partition, s_expected_size, &s_handle);
    if (err != ESP_OK) {
        serial_tx_printf("ERROR fw ota_begin 0x%x\n", (unsigned)err);
        reset_state();
        return;
    }

    ets_sha_enable();
    ets_sha_init(&s_sha_ctx, SHA2_256);
    s_active = true;
    s_written = 0;
    s_next_seq = 0;

    serial_tx_printf("OK fw begin part=%s size=%" PRIu32 "\n",
                     s_partition->label,
                     s_expected_size);
}

static void handle_data(const char *args) {
    if (!s_active) {
        serial_tx_printf("ERROR fw not_active\n");
        return;
    }

    unsigned long seq_ul = 0;
    const char *p = args;
    while (*p == ' ') p++;
    if (sscanf(p, "%lu", &seq_ul) != 1) {
        serial_tx_printf("ERROR fw data usage\n");
        return;
    }
    while (*p && *p != ' ') p++;
    while (*p == ' ') p++;

    if (seq_ul != s_next_seq) {
        serial_tx_printf("ERROR fw seq expected=%" PRIu32 " got=%lu\n", s_next_seq, seq_ul);
        return;
    }

    uint8_t data[384];
    size_t data_len = 0;
    if (!decode_hex_payload(p, data, sizeof(data), &data_len) || data_len == 0) {
        serial_tx_printf("ERROR fw bad_payload seq=%lu\n", seq_ul);
        return;
    }

    if (s_written + data_len > s_expected_size) {
        serial_tx_printf("ERROR fw overflow seq=%lu\n", seq_ul);
        return;
    }

    esp_err_t err = esp_ota_write(s_handle, data, data_len);
    if (err != ESP_OK) {
        serial_tx_printf("ERROR fw write seq=%lu err=0x%x\n", seq_ul, (unsigned)err);
        abort_update();
        return;
    }

    ets_sha_update(&s_sha_ctx, data, data_len, false);
    s_written += (uint32_t)data_len;
    s_next_seq++;

    serial_tx_printf("OK fw data %lu %" PRIu32 "\n", seq_ul, s_written);
}

static void handle_end(void) {
    if (!s_active) {
        serial_tx_printf("ERROR fw not_active\n");
        return;
    }

    if (s_written != s_expected_size) {
        serial_tx_printf("ERROR fw incomplete written=%" PRIu32 " size=%" PRIu32 "\n",
                         s_written,
                         s_expected_size);
        return;
    }

    uint8_t actual_sha[32];
    ets_sha_finish(&s_sha_ctx, actual_sha);
    if (memcmp(actual_sha, s_expected_sha, sizeof(actual_sha)) != 0) {
        serial_tx_printf("ERROR fw sha_mismatch\n");
        abort_update();
        return;
    }

    esp_err_t err = esp_ota_end(s_handle);
    if (err != ESP_OK) {
        serial_tx_printf("ERROR fw ota_end 0x%x\n", (unsigned)err);
        abort_update();
        return;
    }

    err = esp_ota_set_boot_partition(s_partition);
    if (err != ESP_OK) {
        serial_tx_printf("ERROR fw set_boot 0x%x\n", (unsigned)err);
        reset_state();
        return;
    }

    reset_state();
    serial_tx_printf("OK fw reboot\n");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

static void handle_status(void) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    serial_tx_printf("FW: active=%s written=%" PRIu32 " size=%" PRIu32
                     " next_seq=%" PRIu32 " running=%s next=%s\n",
                     s_active ? "Yes" : "No",
                     s_written,
                     s_expected_size,
                     s_next_seq,
                     running ? running->label : "none",
                     next ? next->label : "none");
}

static void handle_version(void) {
    serial_tx_printf(
        "FW_VERSION: Product=%s, Firmware=%s, Hardware=%s, ProjectVer=%s, Release=%s, Git=%s, Dirty=%s, Build=%s, IDF=%s\n",
        OSCORE_PRODUCT_NAME,
        OSCORE_FIRMWARE_NAME,
        OSCORE_HARDWARE_NAME,
        OSCORE_PROJECT_VER,
        OSCORE_RELEASE_DATE,
        OSCORE_GIT_SHA,
        OSCORE_DIRTY_FLAG,
        OSCORE_BUILD_TIME,
        IDF_VER
    );
}

bool fw_update_handle_line(const char *line) {
    if (!line || strncmp(line, "fw", 2) != 0) return false;

    const char *sub = line + 2;
    while (*sub == ' ') sub++;

    if (strncmp(sub, "begin ", 6) == 0) {
        handle_begin(sub + 6);
    } else if (strncmp(sub, "data ", 5) == 0) {
        handle_data(sub + 5);
    } else if (strncmp(sub, "end", 3) == 0) {
        handle_end();
    } else if (strncmp(sub, "abort", 5) == 0) {
        abort_update();
        serial_tx_printf("OK fw abort\n");
    } else if (strncmp(sub, "status", 6) == 0) {
        handle_status();
    } else if (strncmp(sub, "version", 7) == 0) {
        handle_version();
    } else {
        serial_tx_printf("ERROR fw unknown. Use begin|data|end|abort|status|version\n");
    }

    return true;
}
