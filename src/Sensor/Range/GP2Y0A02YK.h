#pragma once

#include <IO/ISPI.h>
#include <Computation/FilterFactory.hpp>
#include <Computation/IFilter.h>
#include <Sensor/ISensor.h>

namespace Sensor {

#define GP2Y0A02YK_BASE                             64
#define GP2Y0A02YK_MAX_RANGE                        .8      //meters
#define GP2Y0A02YK_MIN_RANGE                        .065    //meters
#define GP2Y0A02YK_MAX_DEVIATION                    .02     //meters
#define GP2Y0A02YK_FILTER_SIZE                      10
#define GP2Y0A02YK_READ_DELAY                       10      //microseconds
//-----------------------------------------------------------------------------
// Power regression
// conditions using flat white object
#define GP2Y0A02YK_FIT_ALPHA                        143.39
#define GP2Y0A02YK_FIT_BETA                         -1.181

///****************************************************************************
/// GP2Y0A02YK - Infrared 50-150cm range sensor
///****************************************************************************
class GP2Y0A02YK : public ISensor<double> {
private:
    std::shared_ptr<IO::ISPI>               _spi;
    uint8_t                                 _pin;
    std::unique_ptr<Computation::IFilter>   _filter;
public:
    GP2Y0A02YK(std::shared_ptr<IO::ISPI> spi, uint8_t pin, Computation::FilterFactory &filterFactory);
    virtual void StartCalibration() {}
    virtual void StepCalibration(int step) {}
    virtual void EndCalibration() {}
    virtual double GetReading();
};

};
