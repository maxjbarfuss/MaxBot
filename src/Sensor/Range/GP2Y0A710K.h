#pragma once

#include <IO/ISPI.h>
#include <Computation/FilterFactory.hpp>
#include <Computation/IFilter.h>
#include <Sensor/ISensor.h>

namespace Sensor {

#define GP2Y0A710K_BASE                             64
#define GP2Y0A710K_MAX_RANGE                        2      //meters (datasheet says 5, but data is too noisy after about 2)
#define GP2Y0A710K_MIN_RANGE                        .7     //meters (data sheet says 1.5, but values up to .7 were accurate)
#define GP2Y0A710K_MAX_DEVIATION                    .03    //meters (this is a noisy sensor, so we'll accept more deviation than normal)
#define GP2Y0A710K_FILTER_SIZE                      10
#define GP2Y0A710K_READ_DELAY                       10      //microseconds
//-----------------------------------------------------------------------------
// Power regression 12/20/15 r-squared: .9964 in room lighting
// conditions using flat white object
#define GP2Y0A710K_FIT_ALPHA                        671320
#define GP2Y0A710K_FIT_BETA                         -2.152

///****************************************************************************
/// GP2Y0A710KOF - Infrared 150-500cm range sensor
///****************************************************************************
class GP2Y0A710K : public ISensor<double> {
private:
    std::shared_ptr<IO::ISPI>               _spi;
    uint8_t                                 _pin;
    std::unique_ptr<Computation::IFilter>   _filter;
public:
    GP2Y0A710K(std::shared_ptr<IO::ISPI> spi, uint8_t pin, Computation::FilterFactory &filterFactory);
    virtual void StartCalibration() {}
    virtual void StepCalibration(int step) {}
    virtual void EndCalibration() {}
    virtual double GetReading();
};

};
