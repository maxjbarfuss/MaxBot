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
    std::unique_ptr<IO::II2C> _i2c;
    Eigen::Vector3d _calibration;
    double _scale;

protected:
    void Start() {
        SetScale(L3G4200D_REG_SCALE2000);
        _i2c->Write8(L3G4200D_REG_CTRL1, 0x0F); //power on, enable x,y,z
    }

    void SetScale(uint8_t scale) {
        if (scale == L3G4200D_REG_SCALE250)
            _scale = L3G4200D_VAL_SCALE250;
        else if (scale == L3G4200D_REG_SCALE500)
            _scale = L3G4200D_VAL_SCALE500;
        else
            _scale = L3G4200D_VAL_SCALE2000;
        _i2c->Write8(L3G4200D_REG_CTRL4, scale);
    }

public:
    L3G4200D(IO::I2CFactory& I2CFactory) {
        _i2c = move(I2CFactory.GetI2C(L3G4200D_ADDRESS));
        _scale = 0;
        Start();
    };

    virtual void Calibrate() {
        const int sampleSize = 50;
        _calibration << 0,0,0;
        Eigen::Vector3d min(std::numeric_limits<int>::max(),std::numeric_limits<int>::max(),std::numeric_limits<int>::max());
        Eigen::Vector3d max(std::numeric_limits<int>::min(),std::numeric_limits<int>::min(),std::numeric_limits<int>::min());
        for (int i = 0; i<= sampleSize; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
            auto v = GetReading();
            min(0) = std::min(v(0), min(0));
            max(0) = std::max(v(0), max(0));
            min(1) = std::min(v(1), min(1));
            max(1) = std::max(v(1), max(1));
            min(2) = std::min(v(2), min(2));
            max(2) = std::max(v(2), max(2));
        }
        _calibration << (min(0) + max(0))/2, (min(1) + max(1))/2, (min(2) + max(2))/2;
    }

    Eigen::Vector3d virtual GetReading() {
        auto reading = Eigen::Vector3d(_i2c->Read16Signed(L3G4200D_REG_DATAX),
                                _i2c->Read16Signed(L3G4200D_REG_DATAY),
                                _i2c->Read16Signed(L3G4200D_REG_DATAZ));
        return ((reading) * _scale) - _calibration;
    };
};

};

