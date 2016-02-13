#pragma once

#include <memory>

#include <IMessageBroker.h>
#include <StandardTypes.pb.h>

#include <Publishers/IPublisher.h>
#include <Sensor/ISensor.h>

namespace Publishers {

class VoltagePublisher : IPublisher {
private:
    MaxBotMessages::DoubleStamped                       _v;
    std::shared_ptr<MaxBotMessages::IMessageBroker>     _messageNode;
    std::string                                         _topic;
    std::shared_ptr<Sensor::ISensor<double>>   _sensor;
public:
    VoltagePublisher(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string topic,
                          const std::string hardwareId, std::shared_ptr<Sensor::ISensor<double>> sensor);
    virtual void Publish();
};

};
