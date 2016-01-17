#include <Sensor/IMU/HMC5883L.h>

namespace Sensor {

HMC5883L::HMC5883L(IO::I2CFactory& i2CFactory, uint8_t scale, uint8_t mode) {
    _i2c = move(i2CFactory.GetI2C(HMC5883L_ADDRESS));
    ///TODO: Re-Calibrate
    _hardIronOffset << -51.52,24.84,221.72;
    _softIronScale <<  1,0.980296,1.031088;
    SetScale(scale);
    _i2c->Write8(HMC5883L_REG_MODE, mode);
};

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
    scale &= 0x07; //only lower 3 bits
    _i2c->Write8(HMC5883L_REG_CFGB, 0b00100000);
}

void HMC5883L::StartCalibration() {
    _calibration << 0,0,0;
}

void HMC5883L::StepCalibration(int step) {
    _calibration = ((_calibration * step) + GetReading()) / (step + 1);
}

void HMC5883L::EndCalibration() {}

Eigen::Vector3d HMC5883L::GetReading() {
    Eigen::Vector3d v = Eigen::Vector3d(_i2c->Read16(HMC5883L_REG_DATAX),
                                        _i2c->Read16(HMC5883L_REG_DATAY),
                                        _i2c->Read16(HMC5883L_REG_DATAZ));
    v = ((v * _scale) - _hardIronOffset);
    v(1) *= _softIronScale(1);
    v(2) *= _softIronScale(2);
    return v - _calibration;
};

};
