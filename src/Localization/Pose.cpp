#include <limits>

#include <Localization/Pose.h>

namespace Localization {

#define POSE_COMPONENT_ID "POSE"

Pose::Pose(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string publishPoseTopic, const std::string subscribePoseTopic
, const std::string subscribePositionTopic, const std::string subscribeOrientationTopic)
: _messageNode(messageNode), _publishPoseTopic(publishPoseTopic), _subscribePoseTopic(subscribePoseTopic), _subscribePositionTopic(subscribePositionTopic)
, _subscribeOrientationTopic(subscribeOrientationTopic), _positionFilter(Eigen::Vector3d(0,0,0)), _orientationFilter(Eigen::Quaterniond(1,0,0,0))
{
    _pose.mutable_stamp()->set_component_id(POSE_COMPONENT_ID);
    _messageNode->Subscribe(subscribeOrientationTopic, [&](std::string s){ UpdateOrientation(s); });
    _messageNode->Subscribe(subscribePositionTopic, [&](std::string s){ UpdatePosition(s); });
    _messageNode->Subscribe(subscribePoseTopic, [&](std::string s){ UpdatePose(s); });
}

void Pose::UpdateOrientation(const std::string& message) {
    MaxBotMessages::QuaternionStampedWithAccuracy o;
    o.ParseFromString(message);
    auto q = o.quaternion();
    std::lock_guard<std::mutex> lock(_updateMutex);
    if (o.stamp().measurement_type() == MaxBotMessages::ABSOLUTE) {
        _orientationFilter.AddAbsoluteValue(o.stamp().component_id(), Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()), o.stamp().time(), o.accuracy());
    } else {
        _orientationFilter.AddDifferentialValue(o.stamp().component_id(), Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()), o.stamp().time(), o.accuracy());
    }
}

void Pose::UpdatePosition(const std::string& message) {
    MaxBotMessages::Vector3StampedWithAccuracy pos;
    pos.ParseFromString(message);
    auto v = pos.vector();
    if (pos.stamp().measurement_type() == MaxBotMessages::ABSOLUTE) {
        _positionFilter.AddAbsoluteValue(pos.stamp().component_id(), Eigen::Vector3d(v.x(), v.y(), v.z()), pos.stamp().time(), pos.accuracy());
    } else {
        _positionFilter.AddDifferentialValue(pos.stamp().component_id(), Eigen::Vector3d(v.x(), v.y(), v.z()), pos.stamp().time(), pos.accuracy());
    }
}

void Pose::UpdatePose(const std::string& message) {
    MaxBotMessages::Pose3StampedWithAccuracy pose;
    pose.ParseFromString(message);
    auto v = pose.pose().position();
    auto q = pose.pose().orientation();
    std::lock_guard<std::mutex> lock(_updateMutex);
    if (pose.stamp().measurement_type() == MaxBotMessages::ABSOLUTE) {
        _orientationFilter.AddAbsoluteValue(pose.stamp().component_id(), Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()), pose.stamp().time(), pose.accuracy());
        _positionFilter.AddAbsoluteValue(pose.stamp().component_id(), Eigen::Vector3d(v.x(), v.y(), v.z()), pose.stamp().time(), pose.accuracy());
    } else {
        _orientationFilter.AddDifferentialValue(pose.stamp().component_id(), Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()), pose.stamp().time(), pose.accuracy());
        _positionFilter.AddDifferentialValue(pose.stamp().component_id(), Eigen::Vector3d(v.x(), v.y(), v.z()), pose.stamp().time(), pose.accuracy());
    }
}

void Pose::Publish() {
    auto t = _messageNode->Time();
    auto v1 = _pose.mutable_pose()->mutable_position();
    auto q1 = _pose.mutable_pose()->mutable_orientation();
    {
        std::lock_guard<std::mutex> lock(_updateMutex);
        auto v2 = _positionFilter.GetFilteredValue();
        v1->set_x(v2.x());
        v1->set_y(v2.y());
        v1->set_z(v2.z());
        auto q2 = _orientationFilter.GetFilteredValue();
        q1->set_w(q2.w());
        q1->set_x(q2.x());
        q1->set_y(q2.y());
        q1->set_z(q2.z());
        _positionFilter.Clear(v2);
        _orientationFilter.Clear(q2);
    }
    _pose.mutable_stamp()->set_time(t);
    _messageNode->Publish(_publishPoseTopic, _pose);
}

}

