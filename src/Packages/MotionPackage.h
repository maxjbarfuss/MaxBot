#pragma once

#include <Packages/MaxBotPackageBase.h>
#include <Sensor/ISensor.h>
#include <Subscribers/TeleopSubscriber.h>

namespace Packages {

class MotionPackage : public MaxBotPackageBase {
private:
    std::unique_ptr<Subscribers::TeleopSubscriber> _teleopSubscriber;
public:
    MotionPackage();
};

};
