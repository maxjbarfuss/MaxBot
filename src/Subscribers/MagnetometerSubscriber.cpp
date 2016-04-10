#include <Subscribers/MagnetometerSubscriber.h>

namespace Subscribers {

MagnetometerSubscriber::MagnetometerSubscriber(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string magnetometerTopic, const std::string orientationTopic, std::shared_ptr<Localization::AHRS> ahrs, double accuracy)
: _messageNode(messageNode), _ahrs(ahrs), _lastMeasurementTime(0), _accuracy(accuracy) {
    _messageNode->Subscribe(magnetometerTopic, [&](std::string s){ UpdateMagnetometer(s); });
    _messageNode->Subscribe(orientationTopic, [&](std::string s){ UpdateOrientation(s); });
}

void MagnetometerSubscriber::UpdateMagnetometer(const std::string s) {
    MaxBotMessages::Vector3Stamped magnetometer;
    magnetometer.ParseFromString(s);
    auto v = magnetometer.vector();
    auto time = magnetometer.stamp().microseconds_since_epoch();
    Eigen::Vector3d down = _v;
    Eigen::Vector3d east = down.cross(Eigen::Vector3d(v.x(), v.y(), v.z()));
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
    _lastMeasurementTime = time;
}

void MagnetometerSubscriber::UpdateOrientation(const std::string s) {
//    MaxBotMessages::QuaternionStamped orientation;
//    orientation.ParseFromString(s);
//    auto q = orientation.quaternion();
//    _orientation.w() = q.w();
//    _orientation.x() = q.x();
//    _orientation.y() = q.y();
//    _orientation.z() = q.z();
    MaxBotMessages::Vector3Stamped msg;
    msg.ParseFromString(s);
    auto v = msg.vector();
    _v.x() = v.x();
    _v.y() = v.y();
    _v.z() = v.z();
}

};
