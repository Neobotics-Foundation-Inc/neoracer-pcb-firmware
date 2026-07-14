#include "serial_port.h"

#include "sdkconfig.h"

#if CONFIG_OSCORE_USB_BACKEND_TINYUSB
#include "usb_descriptors.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_cdc_acm.h"
#else
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "freertos/FreeRTOS.h"
#endif

#include <stdio.h>

esp_err_t serial_port_init(void) {
#if CONFIG_OSCORE_USB_BACKEND_TINYUSB
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    tusb_cfg.descriptor.string = oscore_usb_string_descriptor;
    tusb_cfg.descriptor.string_count = oscore_usb_string_descriptor_count;

    esp_err_t err = tinyusb_driver_install(&tusb_cfg);
    if (err != ESP_OK) {
        return err;
    }

    tinyusb_config_cdcacm_t acm_cfg = {};
    acm_cfg.cdc_port = TINYUSB_CDC_ACM_0;
    acm_cfg.callback_rx = NULL;
    acm_cfg.callback_rx_wanted_char = NULL;
    acm_cfg.callback_line_state_changed = NULL;
    acm_cfg.callback_line_coding_changed = NULL;
    return tinyusb_cdcacm_init(&acm_cfg);
#else
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = 256,
        .rx_buffer_size = 2048,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        return err;
    }
    usb_serial_jtag_vfs_use_driver();
    return ESP_OK;
#endif
}

int serial_port_read_bytes(uint8_t *buf, size_t len, uint32_t timeout_ms) {
    if (!buf || len == 0) {
        return 0;
    }

#if CONFIG_OSCORE_USB_BACKEND_TINYUSB
    (void)timeout_ms;
    size_t rx_size = 0;
    esp_err_t err = tinyusb_cdcacm_read(TINYUSB_CDC_ACM_0, buf, len, &rx_size);
    return err == ESP_OK ? (int)rx_size : 0;
#else
    return usb_serial_jtag_read_bytes(buf, len, pdMS_TO_TICKS(timeout_ms));
#endif
}

void serial_port_write_bytes(const char *data, size_t len) {
    if (!data || len == 0) {
        return;
    }

#if CONFIG_OSCORE_USB_BACKEND_TINYUSB
    tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (const uint8_t *)data, len);
    tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
#else
    fwrite(data, 1, len, stdout);
    fflush(stdout);
#endif
}
