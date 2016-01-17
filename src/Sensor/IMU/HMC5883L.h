#pragma once

#include <Dense>

#include <IO/I2CFactory.hpp>
#include <IO/II2C.h>
#include <Sensor/ISensor.h>

namespace Sensor {

//=========================================================================
//  I2C ADDRESS/BITS
#define HMC5883L_ADDRESS                (0x1E)  // Assumes ALT address pin low
//=========================================================================
//  REGISTERS
#define HMC5883L_REG_CFGA               (0x00)
#define HMC5883L_REG_CFGB               (0x01)
#define HMC5883L_REG_MODE               (0x02)
#define HMC5883L_REG_DATAX              (0x03)
#define HMC5883L_REG_DATAY              (0x05)
#define HMC5883L_REG_DATAZ              (0x07)
//=========================================================================
//  CONSTANTS
#define HMC5883L_MODE_CONTINUOUS        (0x00)
#define HMC5883L_MODE_SINGLE            (0x01)
#define HMC5883L_MODE_IDLE              (0x03)
#define HMC5883L_RATE_0_75              (0x00)              //  .75 HZ
#define HMC5883L_RATE_1_5               (0x01)              // 1.5  HZ
#define HMC5883L_RATE_3                 (0x02)              // 3    HZ
#define HMC5883L_RATE_7_5               (0x03)              // 7.5  HZ
#define HMC5883L_RATE_15                (0x04)              //15    Hz
#define HMC5883L_RATE_30                (0x05)              //30    HZ
#define HMC5883L_RATE_75                (0x06)              //75    HZ
#define HMC5883L_SCALE_0_88             (0x00)
#define HMC5883L_SCALE_1_3              (0x01)
#define HMC5883L_SCALE_1_9              (0x02)
#define HMC5883L_SCALE_2_5              (0x03)
#define HMC5883L_SCALE_4_0              (0x04)
#define HMC5883L_SCALE_4_7              (0x05)
#define HMC5883L_SCALE_5_6              (0x06)
#define HMC5883L_SCALE_8_1              (0x07)
#define HMC5883L_GAIN_SCALE_0_88        0.73                //1370 LSB / Gauss
#define HMC5883L_GAIN_SCALE_1_3         0.92                //1090 LSB / Gauss
#define HMC5883L_GAIN_SCALE_1_9         1.22                // 820 LSB / Gauss
#define HMC5883L_GAIN_SCALE_2_5         1.52                // 660 LSB / Gauss
#define HMC5883L_GAIN_SCALE_4_0         2.27                // 440 LSB / Gauss
#define HMC5883L_GAIN_SCALE_4_7         2.56                // 390 LSB / Gauss
#define HMC5883L_GAIN_SCALE_5_6         3.03                // 330 LSB / Gauss
#define HMC5883L_GAIN_SCALE_8_1         4.35                // 230 LSB / Gauss

///****************************************************************************
/// HMC5883L - Magnetometer
///****************************************************************************
class HMC5883L : public ISensor<Eigen::Vector3d> {

private:
    std::unique_ptr<IO::II2C> _i2c;
    double _scale;
    Eigen::Vector3d _hardIronOffset;
    Eigen::Vector3d _softIronScale;
    Eigen::Vector3d _calibration;
private:
    void SetScale(uint8_t scale);
public:
    HMC5883L(IO::I2CFactory& i2CFactory, uint8_t scale, uint8_t rate);
    virtual void StartCalibration();
    virtual void StepCalibration(int step);
    virtual void EndCalibration();
    Eigen::Vector3d virtual GetReading();
};

};
