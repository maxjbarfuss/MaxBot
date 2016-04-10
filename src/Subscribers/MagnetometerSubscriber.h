#pragma once

#include <Dense>

#include <Localization/AHRS.h>

namespace Subscribers {

class MagnetometerSubscriber {
private:
    std::shared_ptr<MaxBotMessages::IMessageBroker> _messageNode;
    std::shared_ptr<Localization::AHRS> _ahrs;
    long long _lastMeasurementTime;
    double _accuracy;
    Eigen::Quaterniond _orientation;
    Eigen::Vector3d _v;
private:
    void UpdateMagnetometer(const std::string s);
    void UpdateOrientation(const std::string s);
public:
    MagnetometerSubscriber(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string magnetometerTopic, const std::string orientationTopic, std::shared_ptr<Localization::AHRS> ahrs, double accuracy);
};

};
