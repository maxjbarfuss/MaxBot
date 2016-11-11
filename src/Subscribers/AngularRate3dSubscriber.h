#pragma once

#include <memory>
#include <string>

#include <IMessageBroker.h>
#include <Pose.pb.h>

namespace Subscribers {

class AngularRate3dSubscriber {
private:
    std::shared_ptr<MaxBotMessages::IMessageBroker> _messageNode;
    std::string _orientationTopic;
    Eigen::Quaterniond _sensorRotation;
    Eigen::Vector3d _sensorTranslation;
    long long _lastMeasurementTime;
    MaxBotMessages::QuaternionStampedWithAccuracy _orientation;
private:
    void UpdateRate(const std::string &s);
public:
    AngularRate3dSubscriber(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string orientationTopic, const std::string rateTopic,
        Eigen::Quaterniond rotation, Eigen::Vector3d translation, double accuracy);
};

};
