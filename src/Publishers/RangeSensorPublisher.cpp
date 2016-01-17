#include <Publishers/RangeSensorPublisher.h>

namespace Publishers {

RangeSensorPublisher::RangeSensorPublisher(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string topic,
const std::string hardwareId, std::shared_ptr<Sensor::ISensor<double>> sensor)
: _messageNode(messageNode), _topic(topic), _sensor(sensor) {
    _range.mutable_stamp()->set_hardware_id(hardwareId);
}

void RangeSensorPublisher::Publish() {
    auto range = _range.mutable_range();
    auto v = _sensor->GetReading();
    _range.mutable_stamp()->set_milliseconds_since_epoch(_messageNode->MillisecondsSinceEpoch());
    range->set_value(v);
    _messageNode->Publish(_topic, _range);
}

};
