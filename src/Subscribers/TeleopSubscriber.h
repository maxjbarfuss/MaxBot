#pragma once

#include <chrono>

#include <IMessageBroker.h>
#include <Velocity.pb.h>

#include <MotionControl/VelocityControl.h>

namespace Subscribers {

#define TELEOP_TIMEOUT  1000    //milliseconds

class TeleopSubscriber {
private:
    std::shared_ptr<MaxBotMessages::IMessageBroker> _messageNode;
    std::string _topic;
    std::shared_ptr<MotionControl::VelocityControl> _velocityControl;
    MaxBotMessages::Velocity2Stamped _velocityMessage;
    std::chrono::high_resolution_clock::time_point _lastCommand;
    bool _active;
private:
    void UpdateVelocity(const std::string message);
    void StartWatchDog();
public:
    TeleopSubscriber(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string topic, std::shared_ptr<MotionControl::VelocityControl> velocityControl);
};

};
