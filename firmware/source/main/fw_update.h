#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool fw_update_handle_line(const char *line);
bool fw_update_active(void);
void fw_update_mark_app_valid(void);

#ifdef __cplusplus
}
#endif
