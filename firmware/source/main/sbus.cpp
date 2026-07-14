#include "sbus.h"
#include "config.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

// SBUS: 100000 baud, 8E2, inverted signal
#define SBUS_BAUD       100000
#define SBUS_FRAME_LEN  25
#define SBUS_HEADER     0x0F
#define SBUS_FOOTER     0x00

static const uart_port_t SBUS_PORT = (uart_port_t)SBUS_UART_NUM;

static void sbus_decode_frame(const uint8_t buf[SBUS_FRAME_LEN], sbus_data_t *out) {
    out->ch[0]  = ((buf[1]       | buf[2]  << 8) & 0x07FF);
    out->ch[1]  = ((buf[2]  >> 3 | buf[3]  << 5) & 0x07FF);
    out->ch[2]  = ((buf[3]  >> 6 | buf[4]  << 2 | buf[5] << 10) & 0x07FF);
    out->ch[3]  = ((buf[5]  >> 1 | buf[6]  << 7) & 0x07FF);
    out->ch[4]  = ((buf[6]  >> 4 | buf[7]  << 4) & 0x07FF);
    out->ch[5]  = ((buf[7]  >> 7 | buf[8]  << 1 | buf[9] << 9) & 0x07FF);
    out->ch[6]  = ((buf[9]  >> 2 | buf[10] << 6) & 0x07FF);
    out->ch[7]  = ((buf[10] >> 5 | buf[11] << 3) & 0x07FF);
    out->ch[8]  = ((buf[12]      | buf[13] << 8) & 0x07FF);
    out->ch[9]  = ((buf[13] >> 3 | buf[14] << 5) & 0x07FF);
    out->ch[10] = ((buf[14] >> 6 | buf[15] << 2 | buf[16] << 10) & 0x07FF);
    out->ch[11] = ((buf[16] >> 1 | buf[17] << 7) & 0x07FF);
    out->ch[12] = ((buf[17] >> 4 | buf[18] << 4) & 0x07FF);
    out->ch[13] = ((buf[18] >> 7 | buf[19] << 1 | buf[20] << 9) & 0x07FF);
    out->ch[14] = ((buf[20] >> 2 | buf[21] << 6) & 0x07FF);
    out->ch[15] = ((buf[21] >> 5 | buf[22] << 3) & 0x07FF);

    out->failsafe  = (buf[23] & (1 << 3)) != 0;
    out->lost_frame = (buf[23] & (1 << 2)) != 0;
}

void sbus_init(void) {
    uart_config_t cfg = {};
    cfg.baud_rate  = SBUS_BAUD;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_EVEN;
    cfg.stop_bits  = UART_STOP_BITS_2;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    uart_driver_install(SBUS_PORT, 256, 0, 0, NULL, 0);
    uart_param_config(SBUS_PORT, &cfg);
    uart_set_pin(SBUS_PORT, SBUS_TX_PIN, SBUS_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // SBUS signal is inverted
    uart_set_line_inverse(SBUS_PORT, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV);
}

bool sbus_read(sbus_data_t *out) {
    static uint8_t frame[SBUS_FRAME_LEN];
    static int frame_pos = 0;

    size_t available = 0;
    uart_get_buffered_data_len(SBUS_PORT, &available);

    while (available > 0) {
        uint8_t byte = 0;
        if (uart_read_bytes(SBUS_PORT, &byte, 1, 0) != 1) {
            break;
        }
        available--;

        if (frame_pos == 0) {
            if (byte != SBUS_HEADER) {
                continue;
            }
            frame[frame_pos++] = byte;
            continue;
        }

        frame[frame_pos++] = byte;

        if (frame_pos == SBUS_FRAME_LEN) {
            bool valid = frame[SBUS_FRAME_LEN - 1] == SBUS_FOOTER;
            if (valid) {
                sbus_decode_frame(frame, out);
                frame_pos = 0;
                return true;
            }

            frame_pos = (byte == SBUS_HEADER) ? 1 : 0;
            if (frame_pos == 1) {
                frame[0] = byte;
            }
        }
    }

    return false;
}
