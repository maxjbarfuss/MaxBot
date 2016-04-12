#pragma once

#include <Localization/AHRS.h>
#include <Packages/MaxBotPackageBase.h>
#include <Subscribers/AngularRate3dSubscriber.h>
#include <Subscribers/ComplimentaryFilterSubscriber.h>

namespace Packages {

class LocalizationPackage : public MaxBotPackageBase {
private:
    std::shared_ptr<Localization::AHRS> _ahrs;
    std::unique_ptr<Subscribers::AngularRate3dSubscriber> _gyro;
    std::unique_ptr<Subscribers::ComplimentaryFilterSubscriber> _complimentaryFilter;
public:
    LocalizationPackage();
};

};
