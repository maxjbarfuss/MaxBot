#pragma once

#include <cmath>
#include <thread>
#include <mutex>

#include <IMessageBroker.h>
#include <Pose.pb.h>
#include <Vector.pb.h>
#include <Velocity.pb.h>

#include <Dense>

#include <Sensor/ISensor.h>

namespace Localization {

class AHRS {
private:
    MaxBotMessages::Vector3Stamped                  _mag;
    MaxBotMessages::Vector3Stamped                  _gyro;
    MaxBotMessages::Vector3Stamped                  _accel;
    MaxBotMessages::Vector3Stamped                  _ahrs;
    MaxBotMessages::Pose2Stamped                    _wheel;
    MaxBotMessages::Velocity2Stamped                _vel;
    std::shared_ptr<MaxBotMessages::IMessageBroker> _messageNode;
    Eigen::Quaterniond                              _orientation;
    Eigen::Quaterniond                              _imuOrientation;
    Eigen::Vector3d                                 _imuOffset;
    double                                          _stepCount;
    Eigen::Quaterniond                              _magnetometer;
    Eigen::Quaterniond                              _calibratedMagnetometer;
    std::string                                     _ahrsTopic;
private:
    void SetMessageVector();
    void UpdateMagnetometer(const std::string message);
    void UpdateAccelerometer(const std::string message);
    void UpdateGyroscope(const std::string message);
    void UpdateWheel(const std::string message);
    void UpdateCommandedVelocity(const std::string message);
    void Calculate();
public:
    AHRS(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, Eigen::Quaterniond imuOrientation, Eigen::Vector3d imuOffset,
         const std::string magnTopic, const std::string gyroTopic, const std::string accelTopic, const std::string wheelTopic, const std::string cvelTopic, const std::string ahrsTopic);
    void Calibrate(std::shared_ptr<Sensor::ISensor<Eigen::Vector3d>> accelerometer, std::shared_ptr<Sensor::ISensor<Eigen::Vector3d>> magnetometer);
    void Publish();
};

};
