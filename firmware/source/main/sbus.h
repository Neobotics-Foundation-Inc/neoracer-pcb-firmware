#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t ch[16];
    bool failsafe;
    bool lost_frame;
} sbus_data_t;

void sbus_init(void);
bool sbus_read(sbus_data_t *out);
