#include <Publishers/VectorSensorPublisher.h>

namespace Publishers {

VectorSensorPublisher::VectorSensorPublisher(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string topic,
const std::string componentId, std::unique_ptr<Sensor::ISensor<Eigen::Vector3d>> sensor)
: _messageNode(messageNode), _topic(topic), _sensor(std::move(sensor)) {
    _v.mutable_stamp()->set_component_id(componentId);
}

void VectorSensorPublisher::Publish() {
    auto vector = _v.mutable_vector();
    auto v = _sensor->GetReading();
    _v.mutable_stamp()->set_time(_messageNode->Time());
    vector->set_x(v(0));
    vector->set_y(v(1));
    vector->set_z(v(2));
    _messageNode->Publish(_topic, _v);
}

};
