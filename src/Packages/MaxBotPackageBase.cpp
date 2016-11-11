#include <algorithm>

#include <MessageBroker.h>

#include <Packages/MaxBotPackageBase.h>

namespace Packages {

MaxBotPackageBase::MaxBotPackageBase() : _messageNode(std::make_shared<MaxBotMessages::MessageBroker>(1)) {}

void MaxBotPackageBase::Start() {
    std::thread t ([this](){
        while (!_stop) {
            int minRemaining = 1000000;
            _messageNode->ProcessSubscriptions();
            _messageNode->ProcessSubscriptions();
            _messageNode->ProcessSubscriptions();
            _messageNode->ProcessSubscriptions();
            _messageNode->ProcessSubscriptions();
            _messageNode->ProcessSubscriptions();
            for (auto & publisher : _publishers) {
                auto elapsed = std::chrono::steady_clock::now() - std::get<2>(publisher);
                auto delay = std::get<1>(publisher);
                if ( elapsed >= delay) {
                    std::get<0>(publisher)->Publish();
                    std::get<2>(publisher) = std::chrono::steady_clock::now();
                }
            }
            for (auto & publisher : _publishers) {
                auto elapsed = std::chrono::steady_clock::now() - std::get<2>(publisher);
                auto delay = std::get<1>(publisher);
                int remaining = (delay - elapsed).count() / 1000;
                minRemaining = std::min(remaining, minRemaining);
            }
            std::this_thread::sleep_for(std::chrono::microseconds(minRemaining));
        }
    });
    t.detach();
}

void MaxBotPackageBase::Stop() {
    _stop = true;
}

};
