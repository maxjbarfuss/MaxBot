#pragma once

#include <functional>

#include <Localization/AHRS.h>

namespace Subscribers {

class AngularRate3dSubscriber {
private:
    std::shared_ptr<MaxBotMessages::IMessageBroker> _messageNode;
    std::string _topic;
    std::shared_ptr<Localization::AHRS> _ahrs;
    double _accuracy;
    MaxBotMessages::Vector3Stamped _msgVector;
private:
    virtual void UpdateRate(const std::string s);
public:
    AngularRate3dSubscriber(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string topic,  std::shared_ptr<Localization::AHRS> ahrs, double accuracy);
};

};
