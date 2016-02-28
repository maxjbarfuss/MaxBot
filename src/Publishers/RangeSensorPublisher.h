#pragma once

#include <memory>

#include <IMessageBroker.h>
#include <Range.pb.h>

#include <Publishers/IPublisher.h>
#include <Sensor/ISensor.h>

namespace Publishers {

class RangeSensorPublisher : public IPublisher {
private:
    MaxBotMessages::RangeStamped                        _range;
    std::shared_ptr<MaxBotMessages::IMessageBroker>     _messageNode;
    std::string                                         _topic;
    std::unique_ptr<Sensor::ISensor<double>>            _sensor;

public:
    RangeSensorPublisher(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string topic,
                         const std::string componentId, std::unique_ptr<Sensor::ISensor<double>> sensor);
    virtual void Publish();
};

};
