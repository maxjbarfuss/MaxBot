#include <Publishers/VoltagePublisher.h>

namespace Publishers {

    VoltagePublisher::VoltagePublisher(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string topic,
                          const std::string hardwareId, std::shared_ptr<Sensor::ISensor<double>> sensor)
    : _messageNode(messageNode), _topic(topic), _sensor(sensor) {
        _v.mutable_stamp()->set_hardware_id(hardwareId);
    }

    void VoltagePublisher::Publish() {
        auto d = _v.mutable_double_();
        d->set_value(_sensor->GetReading());
        _v.mutable_stamp()->set_milliseconds_since_epoch(_messageNode->MillisecondsSinceEpoch());
        _messageNode->Publish(_topic, _v);
    }

}
