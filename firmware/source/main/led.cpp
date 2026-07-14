#include "led.h"
#include "config.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include <string.h>
#include <stdlib.h>

// WS2812B timing at 80 MHz RMT clock (12.5 ns/tick).
#define RMT_RESOLUTION_HZ 80000000UL
#define WS2812_NS_TO_TICKS(ns) (uint16_t)((((uint32_t)(ns)) * (RMT_RESOLUTION_HZ / 1000000UL) + 500UL) / 1000UL)
#define T0H  WS2812_NS_TO_TICKS(400)
#define T0L  WS2812_NS_TO_TICKS(850)
#define T1H  WS2812_NS_TO_TICKS(800)
#define T1L  WS2812_NS_TO_TICKS(450)
#define TRST WS2812_NS_TO_TICKS(50000)

static rmt_channel_handle_t s_tx_chan = NULL;
static rmt_encoder_handle_t s_encoder = NULL;

typedef struct {
    rmt_encoder_t  base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int            state;
    rmt_symbol_word_t reset_code;
} ws2812_encoder_t;

static size_t ws2812_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                             const void *data, size_t size,
                             rmt_encode_state_t *ret_state) {
    ws2812_encoder_t *ws = (ws2812_encoder_t *)encoder;
    rmt_encode_state_t sess = RMT_ENCODING_RESET;
    size_t encoded = 0;

    if (ws->state == 0) {
        encoded += ws->bytes_encoder->encode(ws->bytes_encoder, channel,
                                              data, size, &sess);
        if (sess & RMT_ENCODING_COMPLETE) ws->state = 1;
        if (sess & RMT_ENCODING_MEM_FULL) { *ret_state = RMT_ENCODING_MEM_FULL; return encoded; }
    }
    if (ws->state == 1) {
        encoded += ws->copy_encoder->encode(ws->copy_encoder, channel,
                                             &ws->reset_code, sizeof(rmt_symbol_word_t), &sess);
        if (sess & RMT_ENCODING_COMPLETE) {
            ws->state = RMT_ENCODING_RESET;
            *ret_state = RMT_ENCODING_COMPLETE;
        }
    }
    return encoded;
}

static esp_err_t ws2812_del(rmt_encoder_t *encoder) {
    ws2812_encoder_t *ws = (ws2812_encoder_t *)encoder;
    rmt_del_encoder(ws->bytes_encoder);
    rmt_del_encoder(ws->copy_encoder);
    free(ws);
    return ESP_OK;
}

static esp_err_t ws2812_reset(rmt_encoder_t *encoder) {
    ws2812_encoder_t *ws = (ws2812_encoder_t *)encoder;
    rmt_encoder_reset(ws->bytes_encoder);
    rmt_encoder_reset(ws->copy_encoder);
    ws->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

static esp_err_t ws2812_encoder_new(rmt_encoder_handle_t *ret) {
    ws2812_encoder_t *ws = (ws2812_encoder_t *)calloc(1, sizeof(ws2812_encoder_t));
    if (ws == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ws->base.encode = ws2812_encode;
    ws->base.del    = ws2812_del;
    ws->base.reset  = ws2812_reset;

    // rmt_symbol_word_t layout: duration0(15b), level0(1b), duration1(15b), level1(1b)
    rmt_bytes_encoder_config_t bytes_cfg = {};
    bytes_cfg.bit0.duration0 = T0H; bytes_cfg.bit0.level0 = 1;
    bytes_cfg.bit0.duration1 = T0L; bytes_cfg.bit0.level1 = 0;
    bytes_cfg.bit1.duration0 = T1H; bytes_cfg.bit1.level0 = 1;
    bytes_cfg.bit1.duration1 = T1L; bytes_cfg.bit1.level1 = 0;
    bytes_cfg.flags.msb_first = 1;
    esp_err_t err = rmt_new_bytes_encoder(&bytes_cfg, &ws->bytes_encoder);
    if (err != ESP_OK) {
        free(ws);
        return err;
    }

    rmt_copy_encoder_config_t copy_cfg = {};
    err = rmt_new_copy_encoder(&copy_cfg, &ws->copy_encoder);
    if (err != ESP_OK) {
        rmt_del_encoder(ws->bytes_encoder);
        free(ws);
        return err;
    }

    // reset: low for 50 µs
    ws->reset_code.duration0 = TRST;
    ws->reset_code.level0    = 0;
    ws->reset_code.duration1 = 0;
    ws->reset_code.level1    = 0;

    *ret = &ws->base;
    return ESP_OK;
}

void led_init(void) {
    rmt_tx_channel_config_t tx_cfg = {};
    tx_cfg.gpio_num          = (gpio_num_t)LED_PIN;
    tx_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
    tx_cfg.resolution_hz     = RMT_RESOLUTION_HZ;
    tx_cfg.mem_block_symbols = 64;
    tx_cfg.trans_queue_depth = 4;
    if (rmt_new_tx_channel(&tx_cfg, &s_tx_chan) != ESP_OK) {
        s_tx_chan = NULL;
        return;
    }
    if (ws2812_encoder_new(&s_encoder) != ESP_OK) {
        s_encoder = NULL;
        return;
    }
    if (rmt_enable(s_tx_chan) != ESP_OK) {
        s_tx_chan = NULL;
        s_encoder = NULL;
    }
}

void led_set_color(uint8_t r, uint8_t g, uint8_t b) {
    if (s_tx_chan == NULL || s_encoder == NULL) {
        return;
    }
    uint8_t grb[3] = {g, r, b};
    rmt_transmit_config_t tx_cfg = {};
    tx_cfg.loop_count = 0;
    if (rmt_transmit(s_tx_chan, s_encoder, grb, sizeof(grb), &tx_cfg) == ESP_OK) {
        rmt_tx_wait_all_done(s_tx_chan, 100);
    }
}
