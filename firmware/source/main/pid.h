#pragma once

typedef struct {
    float kp, ki, kd;
    float max_integral;
    float deadband;
    float integral;
    float last_error;
} pid_ctrl_t;

void pid_init(pid_ctrl_t *p, float kp, float ki, float kd,
              float max_integral, float deadband);
int pid_update(pid_ctrl_t *p, float target, float current, int neutral,
               float dt_sec);
void pid_reset(pid_ctrl_t *p);
