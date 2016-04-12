#include <Subscribers/ComplimentaryFilterSubscriber.h>

namespace Subscribers {

ComplimentaryFilterSubscriber::ComplimentaryFilterSubscriber(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string magnetometerTopic,
                                                             const std::string accelerometerTopic, std::shared_ptr<Localization::AHRS> ahrs,
                                                             Eigen::Matrix3d orientation, double accuracy)
: _messageNode(messageNode), _ahrs(ahrs), _orientation(orientation), _accuracy(accuracy) {
    _messageNode->Subscribe(magnetometerTopic, [&](std::string s){ UpdateMagnetometer(s); });
    _messageNode->Subscribe(accelerometerTopic, [&](std::string s){ UpdateAccelerometer(s); });
}

void ComplimentaryFilterSubscriber::UpdateAccelerometer(const std::string s) {
    MaxBotMessages::Vector3Stamped accel;
    accel.ParseFromString(s);
    auto v = accel.vector();
    _accelerometer.x() = v.x();
    _accelerometer.y() = v.y();
    _accelerometer.z() = v.z();
    _accelerometer = _orientation * _accelerometer;
}

void ComplimentaryFilterSubscriber::UpdateMagnetometer(const std::string s) {
    MaxBotMessages::Vector3Stamped magnetometer;
    magnetometer.ParseFromString(s);
    auto v = magnetometer.vector();
    _magnetometer.x() = v.x();
    _magnetometer.y() = v.y();
    _magnetometer.z() = v.z();
    _magnetometer = _orientation * _magnetometer;
}

void ComplimentaryFilterSubscriber::UpdateOrientation() {
    auto time = _messageNode->MicrosecondsSinceEpoch();
    Eigen::Vector3d down = _accelerometer;
    Eigen::Vector3d east = down.cross(_magnetometer);
    Eigen::Vector3d north = east.cross(down);
    down.normalize();
    east.normalize();
    north.normalize();
    Eigen::Matrix3d m;
    m << north(0), north(1), north(2),
         east(0), east(1), east(2),
         down(0), down(1), down(2);
    Eigen::Quaterniond q;
    q = m;
    q.normalize();
    _ahrs->UpdateOrientation(q, time, _accuracy);
}

};
