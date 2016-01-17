#include <Publishers/VectorSensorPublisher.h>

namespace Publishers {

VectorSensorPublisher::VectorSensorPublisher(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string topic,
const std::string hardwareId, std::shared_ptr<Sensor::ISensor<Eigen::Vector3d>> sensor)
: _messageNode(messageNode), _topic(topic), _sensor(sensor) {
    _v.mutable_stamp()->set_hardware_id(hardwareId);
}

void VectorSensorPublisher::Publish() {
    auto vector = _v.mutable_vector();
    auto v = _sensor->GetReading();
    _v.mutable_stamp()->set_milliseconds_since_epoch(_messageNode->MillisecondsSinceEpoch());
    vector->set_x(v(0));
    vector->set_y(v(1));
    vector->set_z(v(2));
    _messageNode->Publish(_topic, _v);
}

};
