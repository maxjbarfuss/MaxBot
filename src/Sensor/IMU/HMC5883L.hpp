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
#define HMC5883L_TOPIC                  "MAGN"

///****************************************************************************
/// HMC5883L - Magnetometer
///****************************************************************************
class HMC5883L : public ISensor<Eigen::Vector3d> {

private:
    std::unique_ptr<IO::II2C> _i2c;
    double _scale;
    Eigen::Vector3d _hardIronOffset;
    Eigen::Vector3d _softIronScale;

    void Start()
    {
        SetScale(HMC5883L_SCALE_1_3);
        _i2c->Write8(HMC5883L_REG_MODE, HMC5883L_MODE_CONTINUOUS);
    }

    void SetScale(uint8_t scale)
    {
        if (scale == HMC5883L_SCALE_0_88) {
            _scale = HMC5883L_GAIN_SCALE_0_88;
        } else if (scale == HMC5883L_SCALE_1_3) {
            _scale = HMC5883L_GAIN_SCALE_1_3;
        } else if (scale == HMC5883L_SCALE_1_9) {
            _scale = HMC5883L_GAIN_SCALE_1_9;
        } else if (scale == HMC5883L_SCALE_2_5) {
            _scale = HMC5883L_GAIN_SCALE_2_5;
        } else if (scale == HMC5883L_SCALE_4_0) {
            _scale = HMC5883L_GAIN_SCALE_4_0;
        } else if (scale == HMC5883L_SCALE_4_7) {
            _scale = HMC5883L_GAIN_SCALE_4_7;
        } else if (scale == HMC5883L_SCALE_5_6) {
            _scale = HMC5883L_GAIN_SCALE_5_6;
        } else {
            _scale = HMC5883L_GAIN_SCALE_8_1;
        }
        scale &= 0x07; //only lower 3 bits
        _i2c->Write8(HMC5883L_REG_CFGB, 0b00100000);
    }

public:
    HMC5883L(IO::I2CFactory& i2CFactory) {
        _i2c = move(i2CFactory.GetI2C(HMC5883L_ADDRESS));
        //calibrated Nov 27th 2015
        _hardIronOffset << -51.52,24.84,221.72;
        _softIronScale <<  1,0.980296,1.031088;
        Start();
    };

    virtual void Calibrate() {
    }

    Eigen::Vector3d virtual GetReading() {
        Eigen::Vector3d v = Eigen::Vector3d(_i2c->Read16(HMC5883L_REG_DATAX),
                              _i2c->Read16(HMC5883L_REG_DATAY),
                              _i2c->Read16(HMC5883L_REG_DATAZ));
        v = ((v * _scale) - _hardIronOffset);
        v(1) *= _softIronScale(1);
        v(2) *= _softIronScale(2);
        return v;
    };
};

};
