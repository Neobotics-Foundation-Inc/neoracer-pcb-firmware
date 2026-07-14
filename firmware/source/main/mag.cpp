#include "mag.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

namespace {
constexpr uint32_t QMC_DEFAULT_READ_TIMEOUT_MS = 0;
constexpr uint32_t QMC_READ_I2C_TIMEOUT_MS = 2;
constexpr uint32_t QMC_CALIBRATION_READ_TIMEOUT_MS = 10;

static const float k_identity[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f,
};
}

QMC6309::QMC6309() {
    memset(hard_iron, 0, sizeof(hard_iron));
    memcpy(soft_iron, k_identity, sizeof(soft_iron));
    for (int i = 0; i < 3; i++) {
        minVal[i] = 1e6f; maxVal[i] = -1e6f;
    }
    sensitivity = SENSITIVITY_8G;
}

void QMC6309::setDevice(i2c_master_dev_handle_t dev) {
    _dev = dev;
}

void QMC6309::writeReg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_master_transmit(_dev, buf, 2, 100);
}

uint8_t QMC6309::readReg(uint8_t reg) {
    uint8_t val = 0xFF;
    readReg(reg, val, 100);
    return val;
}

bool QMC6309::readReg(uint8_t reg, uint8_t &val, uint32_t timeoutMs) {
    esp_err_t ret = i2c_master_transmit_receive(_dev, &reg, 1, &val, 1, timeoutMs);
    return ret == ESP_OK;
}

bool QMC6309::readBurst(uint8_t startReg, uint8_t *buf, uint8_t len) {
    return readBurst(startReg, buf, len, 100);
}

bool QMC6309::readBurst(uint8_t startReg, uint8_t *buf, uint8_t len, uint32_t timeoutMs) {
    esp_err_t ret = i2c_master_transmit_receive(_dev, &startReg, 1, buf, len, timeoutMs);
    return ret == ESP_OK;
}

bool QMC6309::begin(uint8_t rng, uint8_t odr, uint8_t osr1, uint8_t osr2) {
    switch (rng) {
        case 0: sensitivity = SENSITIVITY_32G; break;
        case 1: sensitivity = SENSITIVITY_16G; break;
        default: sensitivity = SENSITIVITY_8G;
    }

    writeReg(QMC_REG_CTRL2, CTRL2_SOFT_RST);
    vTaskDelay(pdMS_TO_TICKS(20));
    writeReg(QMC_REG_CTRL2, 0x00);
    vTaskDelay(pdMS_TO_TICKS(20));

    if (readReg(QMC_REG_CHIP_ID) != 0x90) return false;

    uint8_t ctrl1 = CTRL1_OSR2(osr2) | CTRL1_OSR1(osr1) | CTRL1_MODE(0);
    writeReg(QMC_REG_CTRL1, ctrl1);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t ctrl2 = CTRL2_ODR(odr) | CTRL2_RNG(rng) | CTRL2_SR_MODE(0);
    writeReg(QMC_REG_CTRL2, ctrl2);
    vTaskDelay(pdMS_TO_TICKS(10));

    ctrl1 = (ctrl1 & ~0x03) | CTRL1_MODE(3);
    writeReg(QMC_REG_CTRL1, ctrl1);
    vTaskDelay(pdMS_TO_TICKS(100));

    return true;
}

bool QMC6309::readRaw(int16_t &x, int16_t &y, int16_t &z) {
    return readRaw(x, y, z, QMC_DEFAULT_READ_TIMEOUT_MS);
}

bool QMC6309::readRaw(int16_t &x, int16_t &y, int16_t &z, uint32_t timeoutMs) {
    int64_t start = esp_timer_get_time();
    const int64_t timeoutUs = (int64_t)timeoutMs * 1000;

    while (true) {
        uint8_t status = 0;
        if (!readReg(QMC_REG_STATUS, status, QMC_READ_I2C_TIMEOUT_MS)) return false;
        if (status & STATUS_DRDY) break;
        if (timeoutUs == 0 || esp_timer_get_time() - start >= timeoutUs) return false;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    uint8_t data[6];
    if (!readBurst(QMC_REG_X_LSB, data, 6, QMC_READ_I2C_TIMEOUT_MS)) return false;
    x = (int16_t)((data[1] << 8) | data[0]);
    y = (int16_t)((data[3] << 8) | data[2]);
    z = (int16_t)((data[5] << 8) | data[4]);
    return true;
}

bool QMC6309::readGauss(float &gx, float &gy, float &gz) {
    return readGauss(gx, gy, gz, QMC_DEFAULT_READ_TIMEOUT_MS);
}

bool QMC6309::readGauss(float &gx, float &gy, float &gz, uint32_t timeoutMs) {
    int16_t x, y, z;
    if (!readRaw(x, y, z, timeoutMs)) return false;
    gx = x / sensitivity;
    gy = y / sensitivity;
    gz = z / sensitivity;
    return true;
}

void QMC6309::setCalibration(const float hard[3], const float soft[9]) {
    memcpy(hard_iron, hard, 3 * sizeof(float));
    memcpy(soft_iron, soft, 9 * sizeof(float));
}

void QMC6309::getCalibration(float hard[3], float soft[9]) const {
    memcpy(hard, hard_iron, 3 * sizeof(float));
    memcpy(soft, soft_iron, 9 * sizeof(float));
}

// Hard-iron: min/max center offset.
// Soft-iron: diagonal XY scale only — Z kept at 1.0 since a ground vehicle
// cannot excite the Z axis enough for a reliable range estimate.
void QMC6309::calibrate(uint32_t durationMs) {
    for (int i = 0; i < 3; i++) { minVal[i] = 1e6f; maxVal[i] = -1e6f; }
    int64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < (int64_t)durationMs * 1000) {
        float gx, gy, gz;
        if (readGauss(gx, gy, gz, QMC_CALIBRATION_READ_TIMEOUT_MS)) {
            if (gx < minVal[0]) minVal[0] = gx;
            if (gx > maxVal[0]) maxVal[0] = gx;
            if (gy < minVal[1]) minVal[1] = gy;
            if (gy > maxVal[1]) maxVal[1] = gy;
            if (gz < minVal[2]) minVal[2] = gz;
            if (gz > maxVal[2]) maxVal[2] = gz;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    for (int i = 0; i < 3; i++) {
        hard_iron[i] = (maxVal[i] + minVal[i]) / 2.0f;
    }

    float range_x = maxVal[0] - minVal[0];
    float range_y = maxVal[1] - minVal[1];
    if (range_x < 1e-4f) range_x = 1e-4f;
    if (range_y < 1e-4f) range_y = 1e-4f;
    float avg_range_xy = (range_x + range_y) / 2.0f;

    memcpy(soft_iron, k_identity, sizeof(soft_iron));
    soft_iron[0] = avg_range_xy / range_x;
    soft_iron[4] = avg_range_xy / range_y;
    // soft_iron[8] = 1.0f (Z unchanged)
}

void QMC6309::applyCalibration(float rawX, float rawY, float rawZ,
                                float &calX, float &calY, float &calZ) {
    float dx = rawX - hard_iron[0];
    float dy = rawY - hard_iron[1];
    float dz = rawZ - hard_iron[2];
    // cal = soft_iron * (raw - hard_iron), row-major 3x3
    calX = soft_iron[0]*dx + soft_iron[1]*dy + soft_iron[2]*dz;
    calY = soft_iron[3]*dx + soft_iron[4]*dy + soft_iron[5]*dz;
    calZ = soft_iron[6]*dx + soft_iron[7]*dy + soft_iron[8]*dz;
}

bool QMC6309::readCalibrated(float &calX, float &calY, float &calZ) {
    return readCalibrated(calX, calY, calZ, QMC_DEFAULT_READ_TIMEOUT_MS);
}

bool QMC6309::readCalibrated(float &calX, float &calY, float &calZ, uint32_t timeoutMs) {
    float rawX, rawY, rawZ;
    if (!readGauss(rawX, rawY, rawZ, timeoutMs)) return false;
    applyCalibration(rawX, rawY, rawZ, calX, calY, calZ);
    return true;
}

float QMC6309::calcHeading(float calX, float calY) {
    float heading = atan2f(calY, calX) * 180.0f / 3.14159265358979f;
    if (heading < 0) heading += 360.0f;
    return heading;
}

const char *QMC6309::headingToDirection(float deg) {
    if (deg >= 337.5f || deg < 22.5f)  return "N";
    if (deg < 67.5f)  return "NE";
    if (deg < 112.5f) return "E";
    if (deg < 157.5f) return "SE";
    if (deg < 202.5f) return "S";
    if (deg < 247.5f) return "SW";
    if (deg < 292.5f) return "W";
    return "NW";
}
