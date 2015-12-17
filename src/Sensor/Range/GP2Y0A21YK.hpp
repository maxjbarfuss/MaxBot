#pragma once

#include <iostream>
#include <cmath>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <mcp3004.h>

#include <Computation/FilterFactory.hpp>
#include <Computation/IFilter.h>
#include <Sensor/ISensor.h>

namespace Sensor {

#define GP2Y0A21YK_BASE                             64
#define GP2Y0A21YK_MAX_RANGE                        1       //meters (datasheet says 80cm, but practice says 1m is fairly accurate)
#define GP2Y0A21YK_MIN_RANGE                        .065    //meters (datasheet says 10cm, but practice says 6.5cm is fairly accurate)
#define GP2Y0A21YK_FILTER_SIZE                      4


///****************************************************************************
/// GP2Y0A21YK - Infrared 20-80cm range sensor
///****************************************************************************
class GP2Y0A21YK : public ISensor<double> {
private:
    short                                   _spiChannel;
    short                                   _spiPin;
    std::unique_ptr<Computation::IFilter>   _filter;

public:
    GP2Y0A21YK(short spiChannel, short spiPin, Computation::FilterFactory &filterFactory) : _spiChannel(spiChannel), _spiPin(spiPin) {
        mcp3004Setup (GP2Y0A21YK_BASE, _spiChannel);
        _filter = std::move(filterFactory.GetFilter(GP2Y0A21YK_MIN_RANGE, GP2Y0A21YK_MAX_RANGE, (GP2Y0A21YK_MAX_RANGE - GP2Y0A21YK_MIN_RANGE), GP2Y0A21YK_FILTER_SIZE));
    }

    virtual void Calibrate() {}


    virtual double GetReading() {
        /// magic formula found here: http://home.roboticlab.eu/en/examples/sensor/ir_distance
        return _filter->GetFilteredValue(((5461.0 / (analogRead (GP2Y0A21YK_BASE + _spiPin) - 17) - 2)) / 100.0);
    }
};

};
