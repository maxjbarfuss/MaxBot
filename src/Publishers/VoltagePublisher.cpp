#include <Publishers/VoltagePublisher.h>

namespace Publishers {

    VoltagePublisher::VoltagePublisher(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string topic,
                          const std::string componentId, std::shared_ptr<Sensor::ISensor<double>> sensor)
    : _messageNode(messageNode), _topic(topic), _sensor(sensor) {
        _v.mutable_stamp()->set_component_id(componentId);
    }

    void VoltagePublisher::Publish() {
        auto d = _v.mutable_double_();
        d->set_value(_sensor->GetReading());
        _v.mutable_stamp()->set_microseconds_since_epoch(_messageNode->MicrosecondsSinceEpoch());
        _messageNode->Publish(_topic, _v);
    }

}
