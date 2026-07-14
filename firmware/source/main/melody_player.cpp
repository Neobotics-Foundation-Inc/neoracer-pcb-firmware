#include "melody_player.h"

#include "buzzer.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

typedef enum {
    MELODY_HOST_CONNECTED = 1,
    MELODY_HOST_LOST,
} melody_id_t;

typedef struct {
    uint16_t freq_hz;
    uint16_t duration_ms;
    uint16_t gap_ms;
} melody_note_t;

static const melody_note_t k_host_connected[] = {
    {523, 80, 40},
    {659, 80, 40},
    {784, 140, 0},
};

static const melody_note_t k_host_lost[] = {
    {392, 100, 50},
    {262, 160, 0},
};

static QueueHandle_t s_queue = nullptr;
static int64_t s_last_host_connected_us = 0;
static int64_t s_last_host_lost_us = 0;

static bool rate_limited(int64_t *last_us) {
    const int64_t now = esp_timer_get_time();
    if (*last_us != 0 && now - *last_us < 5000000LL) {
        return true;
    }
    *last_us = now;
    return false;
}

static void play_notes(const melody_note_t *notes, int count) {
    for (int i = 0; i < count; i++) {
        buzzer_tone(notes[i].freq_hz, notes[i].duration_ms);
        vTaskDelay(pdMS_TO_TICKS(notes[i].duration_ms + notes[i].gap_ms));
    }
}

static void task_melody(void *) {
    melody_id_t id;
    while (1) {
        if (xQueueReceive(s_queue, &id, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (id == MELODY_HOST_CONNECTED) {
            play_notes(k_host_connected, (int)(sizeof(k_host_connected) / sizeof(k_host_connected[0])));
        } else if (id == MELODY_HOST_LOST) {
            play_notes(k_host_lost, (int)(sizeof(k_host_lost) / sizeof(k_host_lost[0])));
        }
    }
}

void melody_player_init(void) {
    if (s_queue != nullptr) {
        return;
    }
    s_queue = xQueueCreate(4, sizeof(melody_id_t));
    if (s_queue != nullptr) {
        xTaskCreate(task_melody, "melody", 2048, NULL, 1, NULL);
    }
}

void melody_play_host_connected(void) {
    if (s_queue == nullptr || rate_limited(&s_last_host_connected_us)) {
        return;
    }
    melody_id_t id = MELODY_HOST_CONNECTED;
    xQueueSend(s_queue, &id, 0);
}

void melody_play_host_lost(void) {
    if (s_queue == nullptr || rate_limited(&s_last_host_lost_us)) {
        return;
    }
    melody_id_t id = MELODY_HOST_LOST;
    xQueueSend(s_queue, &id, 0);
}
