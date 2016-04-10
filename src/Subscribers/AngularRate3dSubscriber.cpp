#include <Dense>

#include <Subscribers/AngularRate3dSubscriber.h>

namespace Subscribers {

AngularRate3dSubscriber::AngularRate3dSubscriber(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string topic,  std::shared_ptr<Localization::AHRS> ahrs, double accuracy)
: _messageNode(messageNode), _ahrs(ahrs), _lastMeasurementTime(0), _accuracy(accuracy) {
    _messageNode->Subscribe(topic, [&](std::string s){ UpdateRate(s); });
}

void AngularRate3dSubscriber::UpdateRate(const std::string s) {
    MaxBotMessages::Vector3Stamped msgVector;
    msgVector.ParseFromString(s);
    auto v = msgVector.vector();
    auto time = msgVector.stamp().microseconds_since_epoch();
    _ahrs->UpdateAngularRate(Eigen::Vector3d(v.x(), v.y(), v.z()), time - _lastMeasurementTime, time, _accuracy);
    _lastMeasurementTime = time;
}

};
