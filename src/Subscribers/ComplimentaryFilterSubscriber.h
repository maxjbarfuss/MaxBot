#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <Dense>

#include <IMessageBroker.h>
#include <Pose.pb.h>

namespace Subscribers {

class ComplimentaryFilterSubscriber {
private:
    std::shared_ptr<MaxBotMessages::IMessageBroker> _messageNode;
    std::string _orientationTopic;
    Eigen::Quaterniond _sensorRotation;
    Eigen::Vector3d _sensorTranslation;
    Eigen::Vector3d _accelerometer;
    Eigen::Vector3d _magnetometer;
    MaxBotMessages::QuaternionStampedWithAccuracy _orientation;
    std::mutex _updateMutex;
private:
    void UpdateMagnetometer(const std::string &s);
    void UpdateAccelerometer(const std::string &s);
    void UpdateOrientation();
public:
    ComplimentaryFilterSubscriber(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, std::string orientationTopic, const std::string magnetometerTopic,
       const std::string accelerometerTopic, Eigen::Quaterniond rotation, Eigen::Vector3d translation, double accuracy);
};

};
