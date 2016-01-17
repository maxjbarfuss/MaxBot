#include <Publishers/MotionPublisher.h>

namespace Publishers {

MotionPublisher::MotionPublisher(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string topic,
const std::string hardwareId, std::shared_ptr<MotionControl::VelocityControl> velocityControl)
: _messageNode(messageNode), _topic(topic), _velocityControl(velocityControl) {
    _pose.mutable_stamp()->set_hardware_id(hardwareId);
}

void MotionPublisher::Publish() {
    double x=0, y=0, heading=0;
    _velocityControl->RunMotors(x, y, heading);
    _pose.mutable_stamp()->set_milliseconds_since_epoch(_messageNode->MillisecondsSinceEpoch());
    auto pose = _pose.mutable_pose();
    pose->set_x(_x += x);
    pose->set_y(_y += y);
    pose->set_heading(_heading += heading);
    _messageNode->Publish(_topic, _pose);
}

};
