#include "serial_tx.h"
#include "serial_port.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define SERIAL_TX_LINE_MAX 192
#define SERIAL_TX_QUEUE_LEN 128
#define SERIAL_TX_REALTIME_BUDGET 4

typedef struct {
    char line[SERIAL_TX_LINE_MAX];
} serial_tx_msg_t;

static QueueHandle_t s_serial_tx_queue = NULL;
static TaskHandle_t s_serial_tx_task = NULL;
static portMUX_TYPE s_serial_tx_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_serial_tx_drops = 0;
static serial_tx_msg_t s_realtime_msg[SERIAL_RT_COUNT];
static bool s_realtime_pending[SERIAL_RT_COUNT] = { false };
static int s_realtime_next = 0;

static bool serial_tx_pop_realtime(serial_tx_msg_t *msg) {
    if (!msg) {
        return false;
    }

    taskENTER_CRITICAL(&s_serial_tx_mux);
    for (int i = 0; i < SERIAL_RT_COUNT; ++i) {
        int idx = (s_realtime_next + i) % SERIAL_RT_COUNT;
        if (s_realtime_pending[idx]) {
            *msg = s_realtime_msg[idx];
            s_realtime_pending[idx] = false;
            s_realtime_next = (idx + 1) % SERIAL_RT_COUNT;
            taskEXIT_CRITICAL(&s_serial_tx_mux);
            return true;
        }
    }
    taskEXIT_CRITICAL(&s_serial_tx_mux);
    return false;
}

static void serial_tx_task_fn(void *arg) {
    serial_tx_msg_t msg;
    int realtime_sent = 0;

    while (1) {
        bool has_msg = false;

        if (realtime_sent >= SERIAL_TX_REALTIME_BUDGET &&
            xQueueReceive(s_serial_tx_queue, &msg, 0) == pdTRUE) {
            realtime_sent = 0;
            has_msg = true;
        } else if (serial_tx_pop_realtime(&msg)) {
            realtime_sent++;
            has_msg = true;
        } else if (xQueueReceive(s_serial_tx_queue, &msg, pdMS_TO_TICKS(1)) == pdTRUE) {
            realtime_sent = 0;
            has_msg = true;
        }

        if (has_msg) {
            serial_port_write_bytes(msg.line, strlen(msg.line));
        }
    }
}

void serial_tx_init(void) {
    if (s_serial_tx_queue != NULL) {
        return;
    }

    s_serial_tx_queue = xQueueCreate(
        SERIAL_TX_QUEUE_LEN,
        sizeof(serial_tx_msg_t)
    );

    if (s_serial_tx_queue == NULL) {
        return;
    }

    xTaskCreatePinnedToCore(
        serial_tx_task_fn,
        "serial_tx",
        4096,
        NULL,
        1,
        &s_serial_tx_task,
        0
    );
}

void serial_tx_write_line(const char *line) {
    if (!line) {
        return;
    }

    if (s_serial_tx_queue == NULL) {
        serial_tx_init();
    }

    if (s_serial_tx_queue == NULL) {
        return;
    }

    serial_tx_msg_t msg;
    memset(&msg, 0, sizeof(msg));

    strncpy(msg.line, line, SERIAL_TX_LINE_MAX - 1);

    if (xQueueSend(s_serial_tx_queue, &msg, 0) != pdTRUE) {
        taskENTER_CRITICAL(&s_serial_tx_mux);
        s_serial_tx_drops++;
        taskEXIT_CRITICAL(&s_serial_tx_mux);
    }
}

void serial_tx_printf(const char *fmt, ...) {
    if (!fmt) {
        return;
    }

    char buf[SERIAL_TX_LINE_MAX];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    serial_tx_write_line(buf);
}

void serial_tx_printf_realtime(serial_rt_kind_t kind, const char *fmt, ...) {
    if (!fmt || kind < 0 || kind >= SERIAL_RT_COUNT) {
        return;
    }

    if (s_serial_tx_queue == NULL) {
        serial_tx_init();
    }

    if (s_serial_tx_queue == NULL) {
        return;
    }

    serial_tx_msg_t msg;
    memset(&msg, 0, sizeof(msg));

    va_list args;
    va_start(args, fmt);
    vsnprintf(msg.line, sizeof(msg.line), fmt, args);
    va_end(args);

    taskENTER_CRITICAL(&s_serial_tx_mux);
    s_realtime_msg[kind] = msg;
    s_realtime_pending[kind] = true;
    taskEXIT_CRITICAL(&s_serial_tx_mux);
}

uint32_t serial_tx_drop_count(void) {
    taskENTER_CRITICAL(&s_serial_tx_mux);
    uint32_t drops = s_serial_tx_drops;
    taskEXIT_CRITICAL(&s_serial_tx_mux);
    return drops;
}
