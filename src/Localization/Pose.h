#pragma once

#include <mutex>

#include <IMessageBroker.h>
#include <Pose.pb.h>

#include <Dense>

#include <Computation/WeightedAverageFilter.hpp>
#include <Localization/Pose.h>
#include <Publishers/IPublisher.h>

namespace Localization {

class Pose : public Publishers::IPublisher {
private:
    std::shared_ptr<MaxBotMessages::IMessageBroker> _messageNode;
    std::string _publishPoseTopic;
    std::string _subscribePoseTopic;
    std::string _subscribePositionTopic;
    std::string _subscribeOrientationTopic;
    MaxBotMessages::Pose3Stamped _pose;
    std::mutex _updateMutex;
    Computation::WeightedAverageVectorFilter _positionFilter;
    Computation::WeightedAverageQuaternionFilter _orientationFilter;
private:
    void UpdateOrientation(const std::string& message);
    void UpdatePosition(const std::string& message);
    void UpdatePose(const std::string&  message);
public:
    Pose(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string poseTopic, const std::string subscribePoseTopic,
         const std::string subscribePositionTopic, const std::string subscribeOrientationTopic);
    void Publish();
};

}
