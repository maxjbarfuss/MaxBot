#pragma once

#include <IMessageBroker.h>
#include <Dense>

#include <Sensor/ISensor.h>

namespace Publishers {

class IPublisher {
public:
    virtual void Publish() = 0;
    virtual ~IPublisher() {}
};

};
