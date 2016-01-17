#pragma once

namespace Sensor {

template <typename T>
class ISensor {
public:
    virtual ~ISensor() {}
    virtual void StartCalibration() = 0;
    virtual void StepCalibration(int step) = 0;
    virtual void EndCalibration() = 0;
    virtual T GetReading() = 0;
protected:
private:
};

};


