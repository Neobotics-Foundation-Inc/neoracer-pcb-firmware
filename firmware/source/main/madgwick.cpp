//=============================================================================================
// Madgwick.cpp - Madgwick IMU/AHRS algorithm
// Original: SOH Madgwick, http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
// License: GNU GPL
//=============================================================================================
#include "madgwick.h"
#include <math.h>
#include <esp_timer.h>   // esp_timer_get_time() returns int64_t microseconds

static constexpr float kMinNormSq = 1.0e-12f;

Madgwick::Madgwick() {
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    now = esp_timer_get_time();
    last_update = now;
}

void Madgwick::update(float gx, float gy, float gz,
                      float ax, float ay, float az,
                      float mx, float my, float mz) {
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        updateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    now = esp_timer_get_time();
    delta_t = (now - last_update) / 1000000.0f;
    last_update = now;

    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        float recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        if (recipNorm > 0.0f) {
            ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

            recipNorm = invSqrt(mx*mx + my*my + mz*mz);
        }
        if (recipNorm > 0.0f) {
            mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

            float _2q0mx = 2.0f*q0*mx, _2q0my = 2.0f*q0*my, _2q0mz = 2.0f*q0*mz;
            float _2q1mx = 2.0f*q1*mx;
            float _2q0 = 2.0f*q0, _2q1 = 2.0f*q1, _2q2 = 2.0f*q2, _2q3 = 2.0f*q3;
            float _2q0q2 = 2.0f*q0*q2, _2q2q3 = 2.0f*q2*q3;
            float q0q0=q0*q0, q0q1=q0*q1, q0q2=q0*q2, q0q3=q0*q3;
            float q1q1=q1*q1, q1q2=q1*q2, q1q3=q1*q3;
            float q2q2=q2*q2, q2q3=q2*q3, q3q3=q3*q3;

            float hx = mx*q0q0 - _2q0my*q3 + _2q0mz*q2 + mx*q1q1 + 2.0f*q1*my*q2 + 2.0f*q1*mz*q3 - mx*q2q2 - mx*q3q3;
            float hy = _2q0mx*q3 + my*q0q0 - _2q0mz*q1 + _2q1mx*q2 - my*q1q1 + my*q2q2 + 2.0f*q2*mz*q3 - my*q3q3;
            float _2bx = sqrtf(hx*hx + hy*hy);
            float _2bz = -_2q0mx*q2 + _2q0my*q1 + mz*q0q0 + _2q1mx*q3 - mz*q1q1 + 2.0f*q2*my*q3 - mz*q2q2 + mz*q3q3;
            float _4bx = 2.0f*_2bx, _4bz = 2.0f*_2bz;

            float s0 = -_2q2*( 2.0f*q1q3 - _2q0q2 - ax) + _2q1*(2.0f*q0q1 + _2q2q3 - ay)
                       - _2bz*q2*(_2bx*(0.5f-q2q2-q3q3) + _2bz*(q1q3-q0q2) - mx)
                       + (-_2bx*q3+_2bz*q1)*(_2bx*(q1q2-q0q3) + _2bz*(q0q1+q2q3) - my)
                       + _2bx*q2*(_2bx*(q0q2+q1q3) + _2bz*(0.5f-q1q1-q2q2) - mz);
            float s1 =  _2q3*(2.0f*q1q3 - _2q0q2 - ax) + _2q0*(2.0f*q0q1 + _2q2q3 - ay)
                       - 4.0f*q1*(1.0f-2.0f*q1q1-2.0f*q2q2 - az)
                       + _2bz*q3*(_2bx*(0.5f-q2q2-q3q3) + _2bz*(q1q3-q0q2) - mx)
                       + (_2bx*q2+_2bz*q0)*(_2bx*(q1q2-q0q3) + _2bz*(q0q1+q2q3) - my)
                       + (_2bx*q3-_4bz*q1)*(_2bx*(q0q2+q1q3) + _2bz*(0.5f-q1q1-q2q2) - mz);
            float s2 = -_2q0*(2.0f*q1q3 - _2q0q2 - ax) + _2q3*(2.0f*q0q1 + _2q2q3 - ay)
                       - 4.0f*q2*(1.0f-2.0f*q1q1-2.0f*q2q2 - az)
                       + (-_4bx*q2-_2bz*q0)*(_2bx*(0.5f-q2q2-q3q3) + _2bz*(q1q3-q0q2) - mx)
                       + (_2bx*q1+_2bz*q3)*(_2bx*(q1q2-q0q3) + _2bz*(q0q1+q2q3) - my)
                       + (_2bx*q0-_4bz*q2)*(_2bx*(q0q2+q1q3) + _2bz*(0.5f-q1q1-q2q2) - mz);
            float s3 =  _2q1*(2.0f*q1q3 - _2q0q2 - ax) + _2q2*(2.0f*q0q1 + _2q2q3 - ay)
                       + (-_4bx*q3+_2bz*q1)*(_2bx*(0.5f-q2q2-q3q3) + _2bz*(q1q3-q0q2) - mx)
                       + (-_2bx*q0+_2bz*q2)*(_2bx*(q1q2-q0q3) + _2bz*(q0q1+q2q3) - my)
                       + _2bx*q1*(_2bx*(q0q2+q1q3) + _2bz*(0.5f-q1q1-q2q2) - mz);

            recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
            if (recipNorm > 0.0f) {
                s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

                qDot1 -= beta*s0; qDot2 -= beta*s1; qDot3 -= beta*s2; qDot4 -= beta*s3;
            }
        }
    }

    q0 += qDot1*delta_t; q1 += qDot2*delta_t;
    q2 += qDot3*delta_t; q3 += qDot4*delta_t;

    float recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (recipNorm > 0.0f) {
        q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
    } else {
        q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    }
}

void Madgwick::updateIMU(float gx, float gy, float gz,
                         float ax, float ay, float az) {
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    now = esp_timer_get_time();
    delta_t = (now - last_update) / 1000000.0f;
    last_update = now;

    float qDot1 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float qDot2 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float qDot3 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float qDot4 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        float recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        if (recipNorm > 0.0f) {
            ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

            float _2q0=2.0f*q0, _2q1=2.0f*q1, _2q2=2.0f*q2, _2q3=2.0f*q3;
            float _4q0=4.0f*q0, _4q1=4.0f*q1, _4q2=4.0f*q2;
            float _8q1=8.0f*q1, _8q2=8.0f*q2;
            float q0q0=q0*q0, q1q1=q1*q1, q2q2=q2*q2, q3q3=q3*q3;

            float s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
            float s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
            float s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
            float s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;

            recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
            if (recipNorm > 0.0f) {
                s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

                qDot1 -= beta*s0; qDot2 -= beta*s1; qDot3 -= beta*s2; qDot4 -= beta*s3;
            }
        }
    }

    q0 += qDot1*delta_t; q1 += qDot2*delta_t;
    q2 += qDot3*delta_t; q3 += qDot4*delta_t;

    float recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (recipNorm > 0.0f) {
        q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
    } else {
        q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    }
}

float Madgwick::invSqrt(float x) {
    if (!(x > kMinNormSq) || !isfinite(x)) {
        return 0.0f;
    }
    float root = sqrtf(x);
    if (!(root > 0.0f) || !isfinite(root)) {
        return 0.0f;
    }
    return 1.0f / root;
}
