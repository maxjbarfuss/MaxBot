#include <thread>

#include <Subscribers/TeleopSubscriber.h>

namespace Subscribers {

TeleopSubscriber::TeleopSubscriber(std::shared_ptr<MaxBotMessages::MessageBroker> messageNode, const std::string topic,
std::shared_ptr<MotionControl::VelocityControl> velocityControl)
: _messageNode(messageNode), _topic(topic), _velocityControl(velocityControl), _active(false)  {
    _messageNode->Subscribe(_topic, [&](std::string s){ UpdateVelocity(s); });
}

void TeleopSubscriber::UpdateVelocity(const std::string message) {
    _lastCommand = std::chrono::high_resolution_clock::now();
    if (!_active) StartWatchDog();
    _velocityMessage.ParseFromString(message);
    _velocityControl->SetVelocity(_velocityMessage.velocity().linear(), _velocityMessage.velocity().angular());
}

void TeleopSubscriber::StartWatchDog() {
    std::thread watchDog ([&]{
        _active = true;
        while(_active) {
            if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - _lastCommand).count() >= TELEOP_TIMEOUT) {
                _velocityControl->SetVelocity(0, 0);
                _active = false;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(TELEOP_TIMEOUT));
            }
        }
    });
    watchDog.detach();
}

};

