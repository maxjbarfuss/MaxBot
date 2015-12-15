#pragma once

#include <IMessageBroker.h>
#include <Range.pb.h>

#include <Sensor/ISensor.h>

namespace Publishers {

class RangeSensorPublisher {
private:
    MaxBotMessages::RangeStamped                        _range;
    std::shared_ptr<MaxBotMessages::IMessageBroker>     _messageNode;
    std::string                                         _topic;
    std::shared_ptr<Sensor::ISensor<double>>            _sensor;

protected:

public:
    RangeSensorPublisher(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string topic,
                          const std::string hardwareId, std::shared_ptr<Sensor::ISensor<double>> sensor)
        : _messageNode(messageNode), _topic(topic), _sensor(sensor) {
            _range.mutable_stamp()->set_hardware_id(hardwareId);
        };

    virtual void Publish() {
        auto range = _range.mutable_range();
        auto v = _sensor->GetReading();
        _range.mutable_stamp()->set_milliseconds_since_epoch(_messageNode->MillisecondsSinceEpoch());
        range->set_value(v);
        _messageNode->Publish(_topic, _range);
   };
};

};
