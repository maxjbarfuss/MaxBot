#pragma once

#include <Dense>

#include <Localization/AHRS.h>

namespace Subscribers {

class ComplimentaryFilterSubscriber {
private:
    std::shared_ptr<MaxBotMessages::IMessageBroker> _messageNode;
    std::shared_ptr<Localization::AHRS> _ahrs;
    Eigen::Matrix3d _orientation;
    double _accuracy;
    Eigen::Vector3d _accelerometer;
    Eigen::Vector3d _magnetometer;
private:
    void UpdateMagnetometer(const std::string s);
    void UpdateAccelerometer(const std::string s);
    void UpdateOrientation();
public:
    ComplimentaryFilterSubscriber(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string magnetometerTopic,
                           const std::string accelerometerTopic, std::shared_ptr<Localization::AHRS> ahrs,
                           Eigen::Matrix3d orientation, double accuracy);
};

};
