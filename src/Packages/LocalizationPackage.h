#pragma once

#include <Localization/AHRS.h>
#include <Packages/MaxBotPackageBase.h>
#include <Subscribers/AngularRate3dSubscriber.h>

namespace Packages {

class LocalizationPackage : public MaxBotPackageBase {
private:
    std::shared_ptr<Localization::AHRS> _ahrs;
    std::unique_ptr<Subscribers::AngularRate3dSubscriber> _gyro;
private:
    static double GetGyroAccuracy();
public:
    LocalizationPackage();
};

};
