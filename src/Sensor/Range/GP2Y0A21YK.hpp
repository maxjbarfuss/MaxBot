#pragma once

#include <iostream>
#include <wiringPi.h>

#include <Sensor/ISensor.h>

namespace Sensor {

#define GP2Y0A21YK_MAX_RANGE                        4.0    //meters
#define GP2Y0A21YK_MIN_RANGE                        .02    //meters
#define GP2Y0A21YK_FILTER_SIZE                      12


///****************************************************************************
/// GP2Y0A21YK - Infrared 20-80cm range sensor
///****************************************************************************
class GP2Y0A21YK : public ISensor<double> {
private:
    short                                               _spiChannel;
    std::array<double, GP2Y0A21YK_FILTER_SIZE>          _filter;


    double Filter(double raw) {
    }

public:
    GP2Y0A21YK(short spiChannel) : _spiChannel(spiChannel) {
    }

    virtual void Calibrate() {}

    virtual double GetReading() {
    }
};

};
