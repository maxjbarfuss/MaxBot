#include <Pose.pb.h>

#include <Subscribers/WheelSpeedSubscriber.h>
#include <iostream>
#include <thread>
#include <chrono>

namespace Subscribers {

#define WHEEL_SPEED_ID "WHEL"

WheelSpeedSubscriber::WheelSpeedSubscriber(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, std::string publishPoseTopic, std::string wheelTopic
, std::string subscribePoseTopic, double accuracy)
: _messageNode(messageNode), _poseTopic(publishPoseTopic)
{
    _poseMessage.mutable_stamp()->set_component_id(WHEEL_SPEED_ID);
    _poseMessage.mutable_stamp()->set_measurement_type(MaxBotMessages::DIFFERENTIAL);
    _poseMessage.set_accuracy(accuracy);
    _messageNode->Subscribe(wheelTopic, [&](std::string s){ UpdateWheel(s); });
    _messageNode->Subscribe(subscribePoseTopic, [&](std::string s){ UpdatePose(s); });
    _currentOrientation.x() = 1;
    _currentOrientation.y() = 0;
    _currentOrientation.z() = 0;
    _currentOrientation.w() = 0;
}

void WheelSpeedSubscriber::UpdatePose(const std::string &message) {
    MaxBotMessages::Pose3Stamped pose;
    pose.ParseFromString(message);
    auto q = pose.pose().orientation();
    auto v = pose.pose().position();
    std::lock_guard<std::mutex> lock(_updateMutex);
    _currentOrientation.x() = q.x();
    _currentOrientation.y() = q.y();
    _currentOrientation.z() = q.z();
    _currentOrientation.w() = q.w();
}

void WheelSpeedSubscriber::UpdateWheel(const std::string &s) {
    MaxBotMessages::Pose2Stamped pos;
    pos.ParseFromString(s);
    Eigen::Vector3d v (pos.pose().x(), pos.pose().y(), 0);
    Eigen::Quaterniond q1;
    {
        std::lock_guard<std::mutex> lock(_updateMutex);
        v = _currentOrientation * v;
        auto up = Eigen::Vector3d(2 * (_currentOrientation.x() * _currentOrientation.z() + _currentOrientation.w() * _currentOrientation.y()),
                                  2 * (_currentOrientation.y() * _currentOrientation.z() - _currentOrientation.w() * _currentOrientation.x()),
                                  1 - 2 * (_currentOrientation.x() * _currentOrientation.x() + _currentOrientation.y() * _currentOrientation.y()));
        Eigen::Matrix3d m;
        m = Eigen::AngleAxisd(pos.pose().heading(), up);
        q1 = m / 2;
        _currentOrientation = m * _currentOrientation;
    }
    _poseMessage.mutable_stamp()->set_time(pos.stamp().time());
    auto q2 = _poseMessage.mutable_pose()->mutable_orientation();
    q2->set_w(q1.w());
    q2->set_x(q1.x());
    q2->set_y(q1.y());
    q2->set_z(q1.z());
    auto p = _poseMessage.mutable_pose()->mutable_position();
    p->set_x(v.x());
    p->set_y(v.y());
    p->set_z(v.z());
    _messageNode->Publish(_poseTopic, _poseMessage);
}

};
