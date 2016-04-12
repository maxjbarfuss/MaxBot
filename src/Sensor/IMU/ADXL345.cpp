#include <thread>
#include <chrono>

#include <Sensor/IMU/ADXL345.h>

namespace Sensor {

ADXL345::ADXL345(IO::I2CFactory& i2CFactory, ADXL345DataRate_t rate, ADXL345Range_t range)
: _scale(1)
{
    _i2c = move(i2CFactory.GetI2C(ADXL345_ADDRESS));
    set_bandwidth(rate);
    set_range(range);
    _i2c->Write8(ADXL345_REG_POWER_CTL, 0x08);
}

void ADXL345::set_range(ADXL345Range_t range) {
    uint8_t value = _i2c->Read8(ADXL345_REG_DATA_FORMAT);
    value &= ~0x0F;
    value |= range;
    value |= 0x08;
    _i2c->Write8(ADXL345_REG_DATA_FORMAT, value);
}

void ADXL345::set_bandwidth(ADXL345DataRate_t bw) {
    _i2c->Write8(ADXL345_REG_BW_RATE, bw);
}

void ADXL345::StartCalibration() {
    _scale = 1;
    _average = GetReading();
}

void ADXL345::StepCalibration(int step) {
    _average = ((_average * step) + GetReading()) / (step + 1);
}

void ADXL345::EndCalibration() {
    _scale = 1 / _average.norm();
}

Eigen::Vector3d ADXL345::GetReading() {
    auto x = _i2c->Read10(ADXL345_REG_DATAX);
    auto y = _i2c->Read10(ADXL345_REG_DATAY);
    auto z = _i2c->Read10(ADXL345_REG_DATAZ);
    return Eigen::Vector3d(x,y,z) * _scale;
}

};
