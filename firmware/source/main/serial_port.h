#pragma once

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

esp_err_t serial_port_init(void);
int serial_port_read_bytes(uint8_t *buf, size_t len, uint32_t timeout_ms);
void serial_port_write_bytes(const char *data, size_t len);
