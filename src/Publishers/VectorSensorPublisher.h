#pragma once

#include <memory>

#include <IMessageBroker.h>
#include <Vector.pb.h>
#include <Dense>

#include <Publishers/IPublisher.h>
#include <Sensor/ISensor.h>

namespace Publishers {

class VectorSensorPublisher : public IPublisher {
private:
    MaxBotMessages::Vector3Stamped                      _v;
    std::shared_ptr<MaxBotMessages::IMessageBroker>     _messageNode;
    std::string                                         _topic;
    std::unique_ptr<Sensor::ISensor<Eigen::Vector3d>>   _sensor;
public:
    VectorSensorPublisher(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string topic,
                          const std::string componentId, std::unique_ptr<Sensor::ISensor<Eigen::Vector3d>> sensor);
    virtual void Publish();
};

};
