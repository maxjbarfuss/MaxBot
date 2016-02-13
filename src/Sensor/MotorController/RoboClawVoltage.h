#pragma once

#include <memory>
#include <cmath>

#include <Sensor/ISensor.h>
#include <MotionControl/IRoboClaw.h>

namespace Sensor {

#define ROBO_CLAW_VOLTAGE_SCALE .1

class RoboClawVoltage : public ISensor<double> {

private:
    std::shared_ptr<MotionControl::IRoboClaw>   _motorController;
    uint8_t                                     _address;
public:
    RoboClawVoltage(std::shared_ptr<MotionControl::IRoboClaw> motorController, int address);
    virtual void StartCalibration() {};
    virtual void StepCalibration(int step) {};
    virtual void EndCalibration() {};
    double virtual GetReading();
};

};
