#pragma once

#include <thread>
#include <vector>
#include <chrono>

#include <MessageBroker.h>

#include <Packages/IMaxBotPackage.h>
#include <Publishers/IPublisher.h>

namespace Packages {

class MaxBotPackageBase : public IMaxBotPackage {
private:
    bool _stop = false;
protected:
    std::shared_ptr<MaxBotMessages::IMessageBroker> _messageNode;
    std::vector<std::tuple<std::shared_ptr<Publishers::IPublisher>,  std::chrono::microseconds, std::chrono::steady_clock::time_point>>  _publishers;
public:
    MaxBotPackageBase();
    virtual void Start();
    virtual void Stop();
};

};
