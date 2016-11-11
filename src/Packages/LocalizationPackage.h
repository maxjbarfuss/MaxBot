#pragma once

#include <Localization/Pose.h>
#include <Packages/MaxBotPackageBase.h>
#include <Subscribers/AngularRate3dSubscriber.h>
#include <Subscribers/ComplimentaryFilterSubscriber.h>
#include <Subscribers/WheelSpeedSubscriber.h>

namespace Packages {

class LocalizationPackage : public MaxBotPackageBase {
private:
    std::shared_ptr<Localization::Pose> _pose;
    std::shared_ptr<Subscribers::AngularRate3dSubscriber> _gyro;
    std::shared_ptr<Subscribers::ComplimentaryFilterSubscriber> _complimentaryFilter;
    std::shared_ptr<Subscribers::WheelSpeedSubscriber> _wheelSpeed;
public:
    LocalizationPackage();
};

};
