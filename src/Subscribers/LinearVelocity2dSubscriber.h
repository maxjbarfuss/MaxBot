#pragma once

#include <Dense>

#include <Localization/AHRS.h>
#include <Localization/Pose.h>

namespace Subscribers {

class LinearVelocity2dSubscriber {
private:
    std::shared_ptr<MaxBotMessages::IMessageBroker> _messageNode;
    std::shared_ptr<Localization::AHRS> _ahrs;
    std::shared_ptr<Localization::Pose> _pose;
    Eigen::Matrix3d _sensorRotation;
    Eigen::Translation3d _sensorTranslation;
    double _accuracy;
private:
    void UpdateMagnetometer(const std::string s);
    void UpdateAccelerometer(const std::string s);
    void UpdateOrientation();
public:
    LinearVelocity2dSubscriber(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string magnetometerTopic,
                           const std::string accelerometerTopic, std::shared_ptr<Localization::AHRS> ahrs, std::shared_ptr<Localization::Pose> pose,
                           Eigen::Matrix3d rotation, Eigen::Translation3d translation, double accuracy);
};

};
