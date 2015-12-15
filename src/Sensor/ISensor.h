#pragma once

namespace Sensor {

template <typename T>
class ISensor {
public:
    virtual ~ISensor() {}
    virtual void Calibrate() = 0;
    virtual T GetReading() = 0;
protected:
private:
};

};


