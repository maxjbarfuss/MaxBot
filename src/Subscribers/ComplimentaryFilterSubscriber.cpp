#include <Subscribers/ComplimentaryFilterSubscriber.h>

namespace Subscribers {

#define COMPLEMENTARY_FILTER_ID "CFIL"

ComplimentaryFilterSubscriber::ComplimentaryFilterSubscriber(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, std::string orientationTopic
, std::string magnetometerTopic, std::string accelerometerTopic, Eigen::Quaterniond rotation, Eigen::Vector3d translation, double accuracy)
: _messageNode(messageNode), _orientationTopic(orientationTopic), _sensorRotation(rotation), _sensorTranslation(translation) {
    _orientation.mutable_stamp()->set_component_id(COMPLEMENTARY_FILTER_ID);
    _orientation.set_accuracy(accuracy);
    _messageNode->Subscribe(magnetometerTopic, [&](std::string s){ UpdateMagnetometer(s); });
    _messageNode->Subscribe(accelerometerTopic, [&](std::string s){ UpdateAccelerometer(s); });
}

void ComplimentaryFilterSubscriber::UpdateAccelerometer(const std::string &s) {
    MaxBotMessages::Vector3Stamped accel;
    accel.ParseFromString(s);
    auto v = accel.vector();
    {
        std::lock_guard<std::mutex> lock(_updateMutex);
        _accelerometer.x() = v.x();
        _accelerometer.y() = v.y();
        _accelerometer.z() = v.z();
        _accelerometer = _sensorRotation * _accelerometer;
    }
    UpdateOrientation();
}

void ComplimentaryFilterSubscriber::UpdateMagnetometer(const std::string &s) {
    MaxBotMessages::Vector3Stamped magnetometer;
    magnetometer.ParseFromString(s);
    auto v = magnetometer.vector();
    {
        std::lock_guard<std::mutex> lock(_updateMutex);
        _magnetometer.x() = v.x();
        _magnetometer.y() = v.y();
        _magnetometer.z() = v.z();
        _magnetometer = _sensorRotation * _magnetometer;
    }
    UpdateOrientation();
}

void ComplimentaryFilterSubscriber::UpdateOrientation() {
    Eigen::Vector3d down;
    Eigen::Vector3d east;
    auto time = _messageNode->Time();
    {
        std::lock_guard<std::mutex> lock(_updateMutex);
        down = _accelerometer;
        east = down.cross(_magnetometer);
    }
    Eigen::Vector3d north = east.cross(down);
    down.normalize();
    east.normalize();
    north.normalize();
    Eigen::Matrix3d m;
    m << north(0), north(1), north(2),
         east(0), east(1), east(2),
         down(0), down(1), down(2);
    Eigen::Quaterniond q1;
    q1 = m;
    q1.normalize();
     _orientation.mutable_stamp()->set_time(time);
    auto q2 = _orientation.mutable_quaternion();
    q2->set_w(q1.w());
    q2->set_x(q1.x());
    q2->set_y(q1.y());
    q2->set_z(q1.z());
    _messageNode->Publish(_orientationTopic, _orientation);
}

};
