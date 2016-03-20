#pragma once

#include <memory>

#include <IMessageBroker.h>
#include <Pose.pb.h>
#include <Velocity.pb.h>

#include <MotionControl/VelocityControl.h>
#include <Publishers/IPublisher.h>

namespace Publishers {

class MotionPublisher : public IPublisher {
private:
    std::shared_ptr<MaxBotMessages::IMessageBroker>     _messageNode;
    std::string                                         _poseTopic;
    std::string                                         _velocityTopic;
    std::shared_ptr<MotionControl::VelocityControl>     _velocityControl;
    MaxBotMessages::Pose2Stamped                        _pose;
    MaxBotMessages::Velocity2Stamped                    _velocity;
public:
    MotionPublisher(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string poseTopic, const std::string commandedVelocityTopic,
                    const std::string componentId, std::shared_ptr<MotionControl::VelocityControl> velocityControl);
    virtual void Publish();
};

};
