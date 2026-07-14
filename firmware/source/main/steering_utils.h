#pragma once

#include "config.h"
#include <math.h>

static inline int steering_clamp_pulse(int pulse) {
    if (pulse < STEERING_MIN) return STEERING_MIN;
    if (pulse > STEERING_MAX) return STEERING_MAX;
    return pulse;
}

static inline float steering_us_per_deg(void) {
    return (float)(STEERING_MAX - STEERING_CENTER) / STEERING_MAX_ANGLE_DEG;
}

static inline int steering_trim_deg_to_pulse_offset(float trim_deg) {
    float clamped = CONSTRAIN(trim_deg, STEERING_TRIM_MIN_DEG, STEERING_TRIM_MAX_DEG);
    return (int)lroundf(clamped * steering_us_per_deg());
}

static inline float steering_trim_us_to_deg(int trim_us) {
    return (float)trim_us / steering_us_per_deg();
}

static inline int steering_trimmed_pulse(int pulse, float trim_deg) {
    return steering_clamp_pulse(pulse + steering_trim_deg_to_pulse_offset(trim_deg));
}
