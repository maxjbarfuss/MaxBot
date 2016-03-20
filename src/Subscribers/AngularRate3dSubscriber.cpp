#include <Dense>

#include <Subscribers/AngularRate3dSubscriber.h>

namespace Subscribers {

AngularRate3dSubscriber::AngularRate3dSubscriber(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string topic,  std::shared_ptr<Localization::AHRS> ahrs, double accuracy)
: _messageNode(messageNode), _topic(topic), _ahrs(ahrs), _accuracy(accuracy) {
    _messageNode->Subscribe(_topic, [&](std::string s){ UpdateRate(s); });
}

void AngularRate3dSubscriber::UpdateRate(const std::string s) {
    _msgVector.ParseFromString(s);
    auto v = _msgVector.vector();
    auto time = _msgVector.stamp().microseconds_since_epoch();
    _ahrs->UpdateAngularRate({v.x(), v.y(), v.z()}, time, _accuracy);
}

};
