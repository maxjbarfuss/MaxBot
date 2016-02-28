#include <thread>
#include <chrono>
#include <random>
#include <cmath>
#include <deque>
#include <algorithm>

#include <Sensor/IMU/L3G4200D.h>

namespace Sensor {

L3G4200D::L3G4200D(IO::I2CFactory& i2CFactory, uint8_t scale) {
    _i2c = move(i2CFactory.GetI2C(L3G4200D_ADDRESS));
    SetScale(scale);
    _i2c->Write8(L3G4200D_REG_CTRL1, 0x0F); //power on, enable x,y,z, 100Hz
}

void L3G4200D::SetScale(uint8_t scale) {
    if (scale == L3G4200D_REG_SCALE250)
        _scale = L3G4200D_VAL_SCALE250;
    else if (scale == L3G4200D_REG_SCALE500)
        _scale = L3G4200D_VAL_SCALE500;
    else
        _scale = L3G4200D_VAL_SCALE2000;
    _i2c->Write8(L3G4200D_REG_CTRL4, scale);
}

void L3G4200D::StartCalibration() {
    _calibration << 0,0,0;
    _average = GetReading();
}

void L3G4200D::StepCalibration(int step) {
    _average = ((_average * step) + GetReading()) / (step + 1);
}

void L3G4200D::EndCalibration() {
    _calibration = _average;
}

Eigen::Vector3d L3G4200D::GetReading() {
    auto reading = Eigen::Vector3d(_i2c->Read16Signed(L3G4200D_REG_DATAX),
                                   _i2c->Read16Signed(L3G4200D_REG_DATAY),
                                   _i2c->Read16Signed(L3G4200D_REG_DATAZ));
    return ((reading) * _scale) - _calibration;
}

};
