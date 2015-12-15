#pragma once

#include <IMessageBroker.h>
#include <Dense>

#include <Sensor/ISensor.h>

namespace Publishers {

class IPublisher {
public:
    virtual ~IPublsher() {}
    virtual void Publish() = 0;
};

};
