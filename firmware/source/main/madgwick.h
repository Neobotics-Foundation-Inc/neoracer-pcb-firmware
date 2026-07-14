//=============================================================================================
// Madgwick.h - Madgwick IMU/AHRS algorithm
// Original: SOH Madgwick, http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
// License: GNU GPL
//=============================================================================================
#pragma once
#include <stdint.h>

class Madgwick {
private:
    float delta_t    = 0.0f;
    int64_t now      = 0;
    int64_t last_update = 0;
    static float invSqrt(float x);
    float beta;
    float q0, q1, q2, q3;

public:
    Madgwick();
    void begin(float confBeta) { beta = confBeta; }
    void update(float gx, float gy, float gz, float ax, float ay, float az,
                float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    float getQuatW() { return q0; }
    float getQuatX() { return q1; }
    float getQuatY() { return q2; }
    float getQuatZ() { return q3; }
};
