#pragma once

#include <chrono>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <wiringPi.h>

#include <Computation/FilterFactory.hpp>
#include <Computation/IFilter.h>
#include <Sensor/ISensor.h>

namespace Sensor {

#define HCSRO4_MIN_READ_DELAY                   60     //milliseconds
#define HCSRO4_TRIGGER_DELAY                    10     //microseconds
#define HCSR04_MICROSECONDS_PER_METER           5800.0
#define HCSR04_MAX_RANGE                        1.5    //meters
#define HCSR04_MIN_RANGE                        .02    //meters
#define HCSR04_MAX_DEVIATION                    .02    //meters
#define HCSR04_FILTER_SIZE                      3

///****************************************************************************
/// HC-SR04 - Ultrasonic 2-400cm range sensor
/// ALL HC-SR04 Sensors must be read on the same thread!
///****************************************************************************
class HCSR04 : public ISensor<double> {

private:
    static std::timed_mutex                         _echoPinMutex;
    static void WaitForInterrupt(void) {
        _echoPinMutex.unlock();
    }

private:
    short                                           _triggerPin;
    short                                           _echoPin;
    std::chrono::high_resolution_clock::time_point  _begin;
    std::chrono::high_resolution_clock::time_point  _end;
    std::unique_ptr<Computation::IFilter>           _filter;

    void RunTimer() {
        _echoPinMutex.try_lock_for(std::chrono::milliseconds(HCSRO4_MIN_READ_DELAY));
        auto t = std::chrono::high_resolution_clock::now();
        if (digitalRead(_echoPin) == 1) {
            _begin = t;
        } else {
            _end = t;
        }
    }

    void AddReading() {
        _echoPinMutex.lock();
        auto n = HCSRO4_MIN_READ_DELAY - std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - _end).count();
        if ( n <= HCSRO4_MIN_READ_DELAY && n > 0 ) {
            delay(n);
        }
        digitalWrite(_triggerPin, HIGH);
        delayMicroseconds(HCSRO4_TRIGGER_DELAY);
        digitalWrite(_triggerPin, LOW);
        RunTimer();
        RunTimer();
        _filter->AddValue(std::chrono::duration_cast<std::chrono::microseconds>(_end - _begin).count() / HCSR04_MICROSECONDS_PER_METER);
        _echoPinMutex.unlock();
    }

public:
    HCSR04(short triggerPin, short echoPin, Computation::FilterFactory &filterFactory) : _triggerPin(triggerPin), _echoPin(echoPin) {
        wiringPiSetup();
        pinMode(_triggerPin, OUTPUT);
        pinMode(_echoPin, INPUT);
        wiringPiISR(_echoPin, INT_EDGE_BOTH, &HCSR04::WaitForInterrupt);
        _filter = std::move(filterFactory.GetFilter(HCSR04_MIN_RANGE, HCSR04_MAX_RANGE, HCSR04_MAX_DEVIATION, HCSR04_FILTER_SIZE));
    }

    virtual void Calibrate() {}

    virtual double GetReading() {
        _filter->Clear();
        for(int i=0; i<HCSR04_FILTER_SIZE; i++) AddReading();
        return _filter->GetFilteredValue();
    }
};

std::timed_mutex HCSR04::_echoPinMutex;

};
