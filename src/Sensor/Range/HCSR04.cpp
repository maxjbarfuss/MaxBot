#include <Sensor/Range/HCSR04.h>

namespace Sensor {

/// Static variables
std::timed_mutex HCSR04::_echoPinMutex;

void HCSR04::WaitForInterrupt(void) {
    _echoPinMutex.unlock();
}

HCSR04::HCSR04(short triggerPin, short echoPin) : _triggerPin(triggerPin), _echoPin(echoPin) {
    wiringPiSetup();
    pinMode(_triggerPin, OUTPUT);
    pinMode(_echoPin, INPUT);
    wiringPiISR(_echoPin, INT_EDGE_BOTH, &HCSR04::WaitForInterrupt);
}

void HCSR04::RunTimer() {
    _echoPinMutex.try_lock_for(std::chrono::milliseconds(HCSRO4_MIN_READ_DELAY));
    auto t = std::chrono::high_resolution_clock::now();
    if (digitalRead(_echoPin) == 1) _begin = t;
    else _end = t;
}

double HCSR04::GetReading() {
    double retVal;
    _echoPinMutex.lock();
    auto n = HCSRO4_MIN_READ_DELAY - std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - _end).count();
    if ( n <= HCSRO4_MIN_READ_DELAY && n > 0 ) delay(n);
    digitalWrite(_triggerPin, HIGH);
    delayMicroseconds(HCSRO4_TRIGGER_DELAY);
    digitalWrite(_triggerPin, LOW);
    RunTimer();
    RunTimer();
    retVal = std::chrono::duration_cast<std::chrono::microseconds>(_end - _begin).count() / HCSR04_MICROSECONDS_PER_METER;
    _echoPinMutex.unlock();
    return retVal;
}

};
