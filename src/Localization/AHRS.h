#pragma once

#include <cmath>
#include <thread>
#include <mutex>
#include <chrono>

#include <IMessageBroker.h>
#include <Pose.pb.h>
#include <Vector.pb.h>
#include <Velocity.pb.h>

#include <Dense>

#include <Publishers/IPublisher.h>
#include <Sensor/ISensor.h>

namespace Localization {

class AHRS : public Publishers::IPublisher {
private:
    MaxBotMessages::Vector3Stamped _mag;
    MaxBotMessages::Vector3Stamped _gyro;
    MaxBotMessages::Vector3Stamped _accel;
    MaxBotMessages::Vector3Stamped _ahrs;
    MaxBotMessages::Pose2Stamped _wheel;
    MaxBotMessages::Velocity2Stamped vel;
    std::shared_ptr<MaxBotMessages::IMessageBroker> _messageNode;
    Eigen::Quaterniond _orientation;
    double _stepCount;
    std::string _ahrsTopic;
    Eigen::Quaterniond _angularRate;
    double _linearRate;
    long _lastMeasurementTime;
    long _lastCalculatedTime;
    double _angularRateAccuracy;
    std::mutex _updateMutex;
private:
    void Calculate();
public:
    AHRS(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, Eigen::Quaterniond imuOrientation, Eigen::Vector3d imuOffset,
         const std::string magnTopic, const std::string gyroTopic, const std::string accelTopic, const std::string wheelTopic, const std::string cvelTopic, const std::string ahrsTopic);
    void Calibrate(std::shared_ptr<Sensor::ISensor<Eigen::Vector3d>> accelerometer, std::shared_ptr<Sensor::ISensor<Eigen::Vector3d>> magnetometer);

    /// ********************************************************
    /// Updates the state of the attitude and heading based on an angular change
    /// All values are in body frame (right hand, x forward, z up), where 0,0 is the control point of the vehicle projected to the ground plane upon which the wheels rest
    /// angle: x,y,z measurement of angular change
    /// measurementTime: number of microseconds since the epoch at the end of the measurement
    /// accuracy: value between 0.0 and 1.0 that represents the probability this measurement is accurate (0-100%)
    /// *********************************************************
    void UpdateAngularRate(const std::array<double, 3> angle, const long measurementTime, double accuracy);

    /// ********************************************************
    /// Updates the state of the attitude and heading based on an absolute orientation
    /// All values are in body frame (right hand, x forward, z up), where 0,0 is the control point of the vehicle projected to the ground plane upon which the wheels rest
    /// angle: x,y,z measurement of absolute orientation
    /// measurementTime: number of microseconds since the epoch at the end of the measurement
    /// accuracy: value between 0.0 and 1.0 that represents the probability this measurement is accurate (0-100%)
    /// Returns: current bias of the measurement
    /// *********************************************************
    std::array<double, 3> UpdateAbsoluteOrientation(const std::array<double, 3> angle, const long measurementTime, const double accuracy);

    void Publish();
};

};
