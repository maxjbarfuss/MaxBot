#pragma once

#include <Localization/AHRS.h>

namespace Subscribers {

class AngularRate3dSubscriber {
private:
    std::shared_ptr<MaxBotMessages::IMessageBroker> _messageNode;
    std::shared_ptr<Localization::AHRS> _ahrs;
    long long _lastMeasurementTime;
    Eigen::Matrix3d _orientation;
    double _accuracy;
private:
    virtual void UpdateRate(const std::string s);
public:
    AngularRate3dSubscriber(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string topic,
                            std::shared_ptr<Localization::AHRS> ahrs, Eigen::Matrix3d orientation, double accuracy);
};

};
