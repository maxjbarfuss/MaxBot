#include <Publishers/RangeSensorPublisher.h>

namespace Publishers {

RangeSensorPublisher::RangeSensorPublisher(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string topic,
const std::string componentId, std::unique_ptr<Sensor::ISensor<double>> sensor)
: _messageNode(messageNode), _topic(topic), _sensor(std::move(sensor)) {
    _range.mutable_stamp()->set_component_id(componentId);
}

void RangeSensorPublisher::Publish() {
    auto range = _range.mutable_range();
    auto v = _sensor->GetReading();
    _range.mutable_stamp()->set_time(_messageNode->Time());
    range->set_value(v);
    _messageNode->Publish(_topic, _range);
}

};
