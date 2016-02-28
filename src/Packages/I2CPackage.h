#pragma once

#include <Dense>

#include <IO/I2CFactory.hpp>
#include <Packages/MaxBotPackageBase.h>
#include <Sensor/ISensor.h>

namespace Packages {

class I2CPackage : public MaxBotPackageBase {
public:
    I2CPackage();
};

};
