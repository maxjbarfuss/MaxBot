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
#include <Computation/ProbabalisticAverageFilter.hpp>

namespace Localization {

class AHRS : public Publishers::IPublisher {
private:
    MaxBotMessages::Vector3Stamped _mag;
    MaxBotMessages::Vector3Stamped _gyro;
    MaxBotMessages::Vector3Stamped _accel;
    MaxBotMessages::QuaternionStamped _ahrs;
    MaxBotMessages::Pose2Stamped _wheel;
    MaxBotMessages::Velocity2Stamped vel;
    std::shared_ptr<MaxBotMessages::IMessageBroker> _messageNode;
    Eigen::Quaterniond _orientation;
    double _stepCount;
    std::string _ahrsTopic;
    Computation::ProbabalisticAverageFilter _angularRateFilter;
    double _linearRate;
    long _lastCalculatedTime;
    std::mutex _updateMutex;
    int _calcCount;
public:
    AHRS(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string ahrsTopic);
    Eigen::Quaterniond GetOrientation();
    void Calculate(long long t);
    void Calibrate(std::shared_ptr<Sensor::ISensor<Eigen::Vector3d>> accelerometer, std::shared_ptr<Sensor::ISensor<Eigen::Vector3d>> magnetometer);
    /// ********************************************************
    /// Updates the state of the attitude and heading based on an angular change
    /// All values are in body frame (right hand, x forward, z up), where 0,0 is the control point of the vehicle projected to the ground plane upon which the wheels rest
    /// angle: x,y,z measurement of angular change
    /// measurementDuration: number of microseconds this measurements represents
    /// measurementTime: number of microseconds since the epoch at the end of the measurement
    /// accuracy: value between 0.0 and 1.0 that represents the probability this measurement is accurate (0-100%)
    /// *********************************************************
    void UpdateAngularRate(const Eigen::Vector3d angle, const long long measurementDuration, const long long measurementTime, double accuracy);
    /// ********************************************************
    /// Updates the state of the attitude and heading based on an absolute orientation
    /// All values are in body frame (right hand, x forward, z up), where 0,0 is the control point of the vehicle projected to the ground plane upon which the wheels rest
    /// orientation: quaternion measurement of absolute orientation
    /// measurementTime: number of microseconds since the epoch at the end of the measurement
    /// accuracy: value between 0.0 and 1.0 that represents the probability this measurement is accurate (0-100%)
    /// *********************************************************
    void UpdateOrientation(const Eigen::Quaterniond orientation, const long long measurementTime, const double accuracy);
    void Publish();
};

};
