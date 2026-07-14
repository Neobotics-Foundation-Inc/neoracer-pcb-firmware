#pragma once

#include "config.h"
#include "serial_tx.h"

#if FW_VERBOSE_LOG
#define FW_LOGF(...) serial_tx_printf(__VA_ARGS__)
#else
#define FW_LOGF(...) do { } while (0)
#endif
