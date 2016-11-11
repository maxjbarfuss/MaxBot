#include <Dense>

#include <Subscribers/AngularRate3dSubscriber.h>

namespace Subscribers {

#define MICROSEC_PER_SEC 1000000.0
#define GYRO_SENSOR_ID "GYRO1"

AngularRate3dSubscriber::AngularRate3dSubscriber(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string orientationTopic
, const std::string rateTopic, Eigen::Quaterniond rotation, Eigen::Vector3d translation, double accuracy)
: _messageNode(messageNode), _orientationTopic(orientationTopic), _sensorRotation(rotation), _sensorTranslation(translation)
, _lastMeasurementTime(0) {
    _orientation.mutable_stamp()->set_component_id(GYRO_SENSOR_ID);
    _orientation.mutable_stamp()->set_measurement_type(MaxBotMessages::DIFFERENTIAL);
    _orientation.set_accuracy(accuracy);
    _messageNode->Subscribe(rateTopic, [&](std::string s){ UpdateRate(s); });
}

void AngularRate3dSubscriber::UpdateRate(const std::string &s) {
    MaxBotMessages::Vector3Stamped msgVector;
    msgVector.ParseFromString(s);
    auto v = msgVector.vector();
    auto time = msgVector.stamp().time();
    if((_lastMeasurementTime > 0) && (time > _lastMeasurementTime)) {
        Eigen::Vector3d angularRate (v.x() / 2, v.y() / 2, v.z() / 2);
        angularRate = _sensorRotation * angularRate;
        angularRate *= ((time - _lastMeasurementTime) / MICROSEC_PER_SEC);
        _orientation.mutable_stamp()->set_time(time);
        auto q2 = _orientation.mutable_quaternion();
        q2->set_w(1);
        q2->set_x(angularRate(0));
        q2->set_y(angularRate(1));
        q2->set_z(angularRate(2));
        _messageNode->Publish(_orientationTopic, _orientation);
    }
    _lastMeasurementTime = time;
}


};
