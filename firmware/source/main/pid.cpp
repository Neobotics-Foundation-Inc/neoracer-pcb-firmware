#include "pid.h"
#include "config.h"
#include <math.h>

void pid_init(pid_ctrl_t *p, float kp, float ki, float kd,
              float max_integral, float deadband) {
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->max_integral = max_integral;
    p->deadband = deadband;
    p->integral = 0.0f;
    p->last_error = 0.0f;
}

int pid_update(pid_ctrl_t *p, float target, float current, int neutral,
               float dt_sec) {
    if (dt_sec <= 0.0f) {
        dt_sec = (float)CONTROL_INTERVAL / 1000.0f;
    }

    float error = target - current;
    if (fabsf(error) < p->deadband) error = 0.0f;

    p->integral += error * dt_sec;
    if (p->integral >  p->max_integral) p->integral =  p->max_integral;
    if (p->integral < -p->max_integral) p->integral = -p->max_integral;

    float d = (error - p->last_error) / dt_sec;
    p->last_error = error;

    float output = p->kp * error + p->ki * p->integral + p->kd * d;
    int result = neutral + (int)output;
    if (result < THROTTLE_MIN) result = THROTTLE_MIN;
    if (result > THROTTLE_MAX) result = THROTTLE_MAX;
    return result;
}

void pid_reset(pid_ctrl_t *p) {
    p->integral = 0.0f;
    p->last_error = 0.0f;
}
