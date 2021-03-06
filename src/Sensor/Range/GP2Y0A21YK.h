#pragma once

#include <IO/ISPI.h>
#include <Computation/IFilter.h>
#include <Sensor/ISensor.h>

namespace Sensor {

#define GP2Y0A21YK_BASE                             64
#define GP2Y0A21YK_MAX_RANGE                        .8      //meters
#define GP2Y0A21YK_MIN_RANGE                        .065    //meters (datasheet says 10cm, but practice says 6.5cm is fairly accurate)
#define GP2Y0A21YK_MAX_DEVIATION                    .02     //meters
#define GP2Y0A21YK_FILTER_SIZE                      10
#define GP2Y0A21YK_READ_DELAY                       10      //microseconds
//-----------------------------------------------------------------------------
// Power regression 12/20/15 r-squared: .9993 in room lighting
// conditions using flat white object
#define GP2Y0A21YK_FIT_ALPHA                        143.39
#define GP2Y0A21YK_FIT_BETA                         -1.181

///****************************************************************************
/// GP2Y0A21YK - Infrared 20-80cm range sensor
///****************************************************************************
class GP2Y0A21YK : public ISensor<double> {
private:
    std::shared_ptr<IO::ISPI> _spi;
    uint8_t _pin;
    std::unique_ptr<Computation::IFilter<double>> _filter;
public:
    GP2Y0A21YK(std::shared_ptr<IO::ISPI> spi, uint8_t pin);
    virtual void StartCalibration() {}
    virtual void StepCalibration(int step) {}
    virtual void EndCalibration() {}
    virtual double GetReading();
};

};
