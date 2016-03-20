#include <Publishers/MotionPublisher.h>

namespace Publishers {

MotionPublisher::MotionPublisher(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string poseTopic, const std::string commandedVelocityTopic,
const std::string componentId, std::shared_ptr<MotionControl::VelocityControl> velocityControl)
: _messageNode(messageNode), _poseTopic(poseTopic), _velocityTopic(commandedVelocityTopic), _velocityControl(velocityControl) {
    _pose.mutable_stamp()->set_component_id(componentId);
    _velocity.mutable_stamp()->set_component_id(componentId);
}

void MotionPublisher::Publish() {
    double x=0, y=0, heading=0, angular=0, linear=0;
    _velocity.mutable_stamp()->set_microseconds_since_epoch(_messageNode->MicrosecondsSinceEpoch());
    _velocityControl->RunMotors(x, y, heading, angular, linear);
    _pose.mutable_stamp()->set_microseconds_since_epoch(_messageNode->MicrosecondsSinceEpoch());
    auto pose = _pose.mutable_pose();
    pose->set_x(x);
    pose->set_y(y);
    pose->set_heading(heading);
    _messageNode->Publish(_poseTopic, _pose);
    auto velocity = _velocity.mutable_velocity();
    velocity->set_angular(angular);
    velocity->set_linear(linear);
    _messageNode->Publish(_velocityTopic, _velocity);
}

};
