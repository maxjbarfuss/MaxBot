#pragma once

#include <memory>

#include <IMessageBroker.h>
#include <Pose.pb.h>

#include <MotionControl/VelocityControl.h>
#include <Publishers/IPublisher.h>

namespace Publishers {

class MotionPublisher : IPublisher {
private:
    std::shared_ptr<MaxBotMessages::IMessageBroker>     _messageNode;
    std::string                                         _topic;
    std::shared_ptr<MotionControl::VelocityControl>     _velocityControl;
    MaxBotMessages::Pose2Stamped                        _pose;
    double                                              _x;
    double                                              _y;
    double                                              _heading;
public:
    MotionPublisher(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string topic,
                    const std::string hardwareId, std::shared_ptr<MotionControl::VelocityControl> velocityControl);
    virtual void Publish();
};

};
