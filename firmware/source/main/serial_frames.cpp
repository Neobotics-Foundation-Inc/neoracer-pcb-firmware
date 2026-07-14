#include "serial_frames.h"

#include "app_state.h"
#include "serial_tx.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include <string.h>

extern portMUX_TYPE g_state_mux;

void serial_frame_send_imu(void) {
    float quat[4];
    float accel[3];
    float gyro[3];

    taskENTER_CRITICAL(&g_state_mux);
    memcpy(quat, g_state.quat, sizeof(quat));
    memcpy(accel, g_state.accel, sizeof(accel));
    memcpy(gyro, g_state.gyro, sizeof(gyro));
    taskEXIT_CRITICAL(&g_state_mux);

    serial_tx_printf_realtime(
        SERIAL_RT_IMU,
        "i %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
        quat[1], quat[2], quat[3], quat[0],
        accel[0], accel[1], accel[2],
        gyro[0], gyro[1], gyro[2]
    );
}

void serial_frame_send_mag(void) {
    float mag[3];

    taskENTER_CRITICAL(&g_state_mux);
    memcpy(mag, g_state.mag, sizeof(mag));
    taskEXIT_CRITICAL(&g_state_mux);

    serial_tx_printf_realtime(
        SERIAL_RT_MAG,
        "m %.4f %.4f %.4f\n",
        mag[0], mag[1], mag[2]
    );
}

void serial_frame_send_odom(void) {
    float pos[3];
    float vel[3];
    float yaw;

    taskENTER_CRITICAL(&g_state_mux);
    memcpy(pos, g_state.odom_pos, sizeof(pos));
    memcpy(vel, g_state.odom_vel, sizeof(vel));
    yaw = g_state.odom_yaw;
    taskEXIT_CRITICAL(&g_state_mux);

    serial_tx_printf_realtime(
        SERIAL_RT_ODOM,
        "o %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
        pos[0],
        pos[1],
        pos[2],
        vel[0],
        vel[1],
        vel[2],
        yaw
    );
}

void serial_frame_send_sync(void) {
    float quat[4];
    float pos[3];
    float vel[3];
    float accel[3];
    float gyro[3];
    float yaw;

    taskENTER_CRITICAL(&g_state_mux);
    memcpy(quat, g_state.quat, sizeof(quat));
    memcpy(pos, g_state.odom_pos, sizeof(pos));
    memcpy(vel, g_state.odom_vel, sizeof(vel));
    memcpy(accel, g_state.accel, sizeof(accel));
    memcpy(gyro, g_state.gyro, sizeof(gyro));
    yaw = g_state.odom_yaw;
    taskEXIT_CRITICAL(&g_state_mux);

    serial_tx_printf_realtime(
        SERIAL_RT_SYNC,
        "s %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
        pos[0], pos[1], pos[2],
        vel[0], vel[1], vel[2],
        yaw,
        quat[1], quat[2], quat[3], quat[0],
        accel[0], accel[1], accel[2],
        gyro[0], gyro[1], gyro[2]
    );
}

void serial_frame_send_battery(void) {
    float voltage = 0.0f;

    taskENTER_CRITICAL(&g_state_mux);
    voltage = g_state.voltage;
    taskEXIT_CRITICAL(&g_state_mux);

    serial_tx_printf("b %.2f\n", voltage);
}
