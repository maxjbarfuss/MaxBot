#include <Sensor/IMU/HMC5883L.h>

namespace Sensor {

HMC5883L::HMC5883L(IO::I2CFactory& i2CFactory, uint8_t scale, uint8_t mode, uint8_t rate, Eigen::Vector3d hardIron, Eigen::Vector3d softIron)
: _hardIronOffset(hardIron), _softIronScale(softIron) {
    _i2c = move(i2CFactory.GetI2C(HMC5883L_ADDRESS));
    SetRate(rate);
    SetScale(scale);
    _i2c->Write8(HMC5883L_REG_MODE, mode);
}

void HMC5883L::SetRate(uint8_t rate)
{
    uint8_t regVal = 0b00011100; //reserved
    if (rate == HMC5883L_RATE_0_75) {
        regVal = 0b00000000;
    } else if (rate == HMC5883L_RATE_1_5) {
        regVal = 0b00000100;
    } else if (rate == HMC5883L_RATE_3) {
        regVal = 0b00001000;
    } else if (rate == HMC5883L_RATE_7_5) {
        regVal = 0b00001100;
    } else if (rate == HMC5883L_RATE_15) {
        regVal = 0b00010000;
    } else if (rate == HMC5883L_RATE_30) {
        regVal = 0b00010100;
    } else if (rate == HMC5883L_RATE_75) {
        regVal = 0b00011000;
    }
    _i2c->Write8(HMC5883L_REG_CFGA, regVal);
}

void HMC5883L::SetScale(uint8_t scale)
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
    scale &= 0x07;
    _i2c->Write8(HMC5883L_REG_CFGB, scale);
}

Eigen::Vector3d HMC5883L::GetReading() {
    //I have no idea why Z and Y are swapped
    Eigen::Vector3d v = Eigen::Vector3d(_i2c->Read16(HMC5883L_REG_DATAX),
                                        _i2c->Read16(HMC5883L_REG_DATAZ),
                                        _i2c->Read16(HMC5883L_REG_DATAY));
    v = ((v * _scale) - _hardIronOffset);
    v(1) *= _softIronScale(1);
    v(2) *= _softIronScale(2);
    return v;
}

};
