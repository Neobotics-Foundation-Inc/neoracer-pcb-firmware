#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"

#define QMC6309_ADDR            0x7C

#define QMC_REG_CHIP_ID         0x00
#define QMC_REG_X_LSB           0x01
#define QMC_REG_STATUS          0x09
#define QMC_REG_CTRL1           0x0A
#define QMC_REG_CTRL2           0x0B

#define STATUS_DRDY             (1 << 0)

#define CTRL1_OSR2(n)           ((n) << 5)
#define CTRL1_OSR1(n)           ((n) << 3)
#define CTRL1_MODE(m)           ((m) & 0x03)

#define CTRL2_SOFT_RST          (1 << 7)
#define CTRL2_ODR(n)            ((n) << 4)
#define CTRL2_RNG(n)            ((n) << 2)
#define CTRL2_SR_MODE(n)        ((n) << 0)

#define SENSITIVITY_32G         1000.0f
#define SENSITIVITY_16G         2000.0f
#define SENSITIVITY_8G          4000.0f

class QMC6309 {
public:
    QMC6309();
    void setDevice(i2c_master_dev_handle_t dev);
    bool begin(uint8_t rng = 2, uint8_t odr = 4, uint8_t osr1 = 3, uint8_t osr2 = 4);
    bool readRaw(int16_t &x, int16_t &y, int16_t &z);
    bool readRaw(int16_t &x, int16_t &y, int16_t &z, uint32_t timeoutMs);
    bool readGauss(float &gx, float &gy, float &gz);
    bool readGauss(float &gx, float &gy, float &gz, uint32_t timeoutMs);

    // cal = soft_iron * (raw - hard_iron)
    void applyCalibration(float rawX, float rawY, float rawZ,
                          float &calX, float &calY, float &calZ);
    bool readCalibrated(float &calX, float &calY, float &calZ);
    bool readCalibrated(float &calX, float &calY, float &calZ, uint32_t timeoutMs);

    // hard[3]: hard-iron offset (Gauss); soft[9]: soft-iron matrix, row-major 3x3
    void setCalibration(const float hard[3], const float soft[9]);
    void getCalibration(float hard[3], float soft[9]) const;

    // Simple min/max hard-iron calibration (quick field use).
    void calibrate(uint32_t durationMs = 30000);

    float calcHeading(float calX, float calY);
    const char *headingToDirection(float deg);

private:
    i2c_master_dev_handle_t _dev = nullptr;
    float sensitivity;
    float hard_iron[3];   // hard-iron offset (Gauss)
    float soft_iron[9];   // soft-iron matrix, row-major 3x3
    float minVal[3], maxVal[3];

    void writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);
    bool readReg(uint8_t reg, uint8_t &val, uint32_t timeoutMs);
    bool readBurst(uint8_t startReg, uint8_t *buf, uint8_t len);
    bool readBurst(uint8_t startReg, uint8_t *buf, uint8_t len, uint32_t timeoutMs);
};
