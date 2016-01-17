#pragma once

#include <Dense>

#include <IO/I2CFactory.hpp>
#include <IO/II2C.h>
#include <Sensor/ISensor.h>

namespace Sensor {

//=========================================================================
//  I2C ADDRESS/BITS
#define L3G4200D_ADDRESS                    (0x69)    // Assumes ALT address pin low
//=========================================================================
//  REGISTERS
#define L3G4200D_REG_CTRL1                  (0x20)
#define L3G4200D_REG_CTRL2                  (0x21)
#define L3G4200D_REG_CTRL3                  (0x22)
#define L3G4200D_REG_CTRL4                  (0x23)
#define L3G4200D_REG_CTRL5                  (0x24)
#define L3G4200D_REG_DATAX                  (0x28)
#define L3G4200D_REG_DATAY                  (0x2A)
#define L3G4200D_REG_DATAZ                  (0x2C)
//=========================================================================
// CONSTANTS
#define L3G4200D_VAL_SCALE250               0.000152716309375 //0.00875 dps -> rad/s
#define L3G4200D_VAL_SCALE500               0.00030543261875  // 0.0175 dps -> rad/s
#define L3G4200D_VAL_SCALE2000              0.001221730475    //   0.07 dps -> rad/s
#define L3G4200D_REG_SCALE250               (0x00)
#define L3G4200D_REG_SCALE500               (0x10)
#define L3G4200D_REG_SCALE2000              (0x30)

///****************************************************************************
/// L3G4200D - Gyroscope
///****************************************************************************
class L3G4200D : public ISensor<Eigen::Vector3d> {

private:
    std::unique_ptr<IO::II2C>   _i2c;
    Eigen::Vector3d             _calibration;
    Eigen::Vector3d             _average;
    double                      _scale;
private:
    void SetScale(uint8_t scale);
public:
    L3G4200D(IO::I2CFactory& i2CFactory, uint8_t scale);
    virtual void StartCalibration();
    virtual void StepCalibration(int step);
    virtual void EndCalibration();
    Eigen::Vector3d virtual GetReading();
};

};

