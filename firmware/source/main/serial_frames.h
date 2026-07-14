#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void serial_frame_send_imu(void);
void serial_frame_send_mag(void);
void serial_frame_send_odom(void);
void serial_frame_send_sync(void);
void serial_frame_send_battery(void);

#ifdef __cplusplus
}
#endif
