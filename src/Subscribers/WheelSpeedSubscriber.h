#pragma once

#include <Dense>

#include <Localization/Pose.h>

namespace Subscribers {

class WheelSpeedSubscriber {
private:
    std::shared_ptr<MaxBotMessages::IMessageBroker> _messageNode;
    std::string _poseTopic;
    std::shared_ptr<Localization::Pose> _pose;
    Eigen::Quaterniond _currentOrientation;
    std::mutex _updateMutex;
    MaxBotMessages::Pose3StampedWithAccuracy _poseMessage;
private:
    void UpdatePose(const std::string &s);
    void UpdateWheel(const std::string &s);
public:
    WheelSpeedSubscriber(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, std::string publishPoseTopic, std::string wheelTopic,
        std::string subscribePoseTopic, double accuracy);
};

};
