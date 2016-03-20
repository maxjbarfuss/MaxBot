#pragma once

#include <memory>
#include <chrono>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <wiringPi.h>

#include <Computation/IFilter.h>
#include <Sensor/ISensor.h>

namespace Sensor {

#define HCSRO4_MIN_READ_DELAY                   60     //milliseconds
#define HCSRO4_TRIGGER_DELAY                    10     //microseconds
#define HCSR04_MICROSECONDS_PER_METER           5800.0
#define HCSR04_MAX_RANGE                        1.5    //meters
#define HCSR04_MIN_RANGE                        .02    //meters
#define HCSR04_MAX_DEVIATION                    .02    //meters

///****************************************************************************
/// HC-SR04 - Ultrasonic 2-400cm range sensor
/// ALL HC-SR04 Sensors must be read on the same thread!
///****************************************************************************
class HCSR04 : public ISensor<double> {

private:
    static std::timed_mutex _echoPinMutex;
    static void WaitForInterrupt(void);
private:
    short _triggerPin;
    short _echoPin;
    std::chrono::high_resolution_clock::time_point _begin;
    std::chrono::high_resolution_clock::time_point _end;
private:
    void RunTimer();
public:
    HCSR04(short triggerPin, short echoPin);
    virtual void StartCalibration() {}
    virtual void StepCalibration(int step) {}
    virtual void EndCalibration() {}
    virtual double GetReading();
};



};
