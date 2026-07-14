#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void serial_tx_init(void);
void serial_tx_printf(const char *fmt, ...);
void serial_tx_write_line(const char *line);

typedef enum {
    SERIAL_RT_SYNC = 0,
    SERIAL_RT_ODOM,
    SERIAL_RT_IMU,
    SERIAL_RT_MAG,
    SERIAL_RT_COUNT
} serial_rt_kind_t;

void serial_tx_printf_realtime(serial_rt_kind_t kind, const char *fmt, ...);
uint32_t serial_tx_drop_count(void);

#ifdef __cplusplus
}
#endif
