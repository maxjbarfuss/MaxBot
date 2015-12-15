#pragma once

#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#include <Sensor/ISensor.h>

namespace Sensor {

#define HCSRO4_MIN_READ_DELAY                   60     //milliseconds
#define HCSRO4_TRIGGER_DELAY                    10     //microseconds
#define HCSR04_MICROSECONDS_PER_METER           5800.0
#define HCSR04_MAX_RANGE                        4.0    //meters
#define HCSR04_MIN_RANGE                        .02    //meters
#define HCSR04_FILTER_SIZE                      12


///****************************************************************************
/// HC-SR04 - Ultrasonic
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
    std::array<double, HCSR04_FILTER_SIZE>          _filter;

    void RunTimer() {
        _echoPinMutex.try_lock_for(std::chrono::milliseconds(HCSRO4_MIN_READ_DELAY));
        auto t = std::chrono::high_resolution_clock::now();
        if (digitalRead(_echoPin) == 1) {
            _begin = t;
        } else {
            _end = t;
        }
    }

    double AverageFilter() {
        double total = 0.0;
        int count = 0;
        for (int i = HCSR04_FILTER_SIZE - 1; i >= 0; i--) {
            if (_filter[i] <= HCSR04_MAX_RANGE && _filter[i] >= HCSR04_MIN_RANGE) {
                    total += _filter[i];
                    count += i;
            }
        }
        return total / count;
    }

    double Filter(double raw) {
        double total = 0.0;
        int count = 0;
        double avg = AverageFilter();
        for (int i = HCSR04_FILTER_SIZE; i >= 0; i--) {
            double v = (i < HCSR04_FILTER_SIZE) ? _filter[i] : raw;
            if (v <= HCSR04_MAX_RANGE && v >= HCSR04_MIN_RANGE
                && v > avg - (HCSR04_MAX_RANGE - HCSR04_MIN_RANGE) / 3
                && v < avg + (HCSR04_MAX_RANGE - HCSR04_MIN_RANGE) / 3) {
                    total += v * i*i;
                    count += i*i;
            }
            if (i < HCSR04_FILTER_SIZE - 1)
                _filter[i] = _filter[i+1];
        }
        _filter[HCSR04_FILTER_SIZE - 1] = raw;
        return total / count;
    }

public:
    HCSR04(short triggerPin, short echoPin) : _triggerPin(triggerPin), _echoPin(echoPin) {
        wiringPiSetup();
        pinMode(_triggerPin, OUTPUT);
        pinMode(_echoPin, INPUT);
        wiringPiISR(_echoPin, INT_EDGE_BOTH, &HCSR04::WaitForInterrupt);
        for (int i = 0; i<HCSR04_FILTER_SIZE; i++)
            _filter[i] = HCSR04_MAX_RANGE + 1;
    }

    virtual void Calibrate() {}

    virtual double GetReading() {
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
        auto m = Filter(std::chrono::duration_cast<std::chrono::microseconds>(_end - _begin).count() / HCSR04_MICROSECONDS_PER_METER);
        std::cout << m << std::endl;
        _echoPinMutex.unlock();
        return m;
    }
};

std::timed_mutex HCSR04::_echoPinMutex;

};
