#include <Dense>

#include <Subscribers/AngularRate3dSubscriber.h>

namespace Subscribers {

AngularRate3dSubscriber::AngularRate3dSubscriber(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string topic,
                                                 std::shared_ptr<Localization::AHRS> ahrs, Eigen::Matrix3d orientation, double accuracy)
: _messageNode(messageNode), _ahrs(ahrs), _lastMeasurementTime(0), _orientation(orientation), _accuracy(accuracy) {
    _messageNode->Subscribe(topic, [&](std::string s){ UpdateRate(s); });
}

void AngularRate3dSubscriber::UpdateRate(const std::string s) {
    MaxBotMessages::Vector3Stamped msgVector;
    msgVector.ParseFromString(s);
    auto v = msgVector.vector();
    auto time = msgVector.stamp().microseconds_since_epoch();
    Eigen::Vector3d gyro (v.x(), v.y(), v.z());
    gyro = _orientation * gyro;
    _ahrs->UpdateAngularRate(gyro, time - _lastMeasurementTime, time, _accuracy);
    _lastMeasurementTime = time;
}

};
