#include <Localization/AHRS.h>

namespace Localization {

AHRS::AHRS(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, Eigen::Quaterniond imuOrientation, Eigen::Vector3d imuOffset,
     const std::string magnTopic, const std::string gyroTopic, const std::string accelTopic, const std::string wheelTopic, const std::string cvelTopic, const std::string ahrsTopic)
: _messageNode(messageNode), _stepCount(0), _ahrsTopic(ahrsTopic) {
    _ahrs.mutable_stamp()->set_component_id("AHRS");
    _messageNode->Subscribe(magnTopic, [this](std::string s){ UpdateMagnetometer(s); });
    _messageNode->Subscribe(accelTopic, [&](std::string s){ UpdateAccelerometer(s); });
    _messageNode->Subscribe(gyroTopic, [&](std::string s){ UpdateGyroscope(s); });
    _messageNode->Subscribe(wheelTopic, [&](std::string s){ UpdateWheel(s); });
    _messageNode->Subscribe(cvelTopic, [&](std::string s){ UpdateCommandedVelocity(s); });
}

void AHRS::SetMessageVector()
{
    _ahrs.mutable_stamp()->set_milliseconds_since_epoch(_messageNode->MillisecondsSinceEpoch());
    auto v = _ahrs.mutable_vector();
    Eigen::Quaterniond q = _imuOrientation * _orientation * _imuOrientation.inverse();
    Eigen::Vector3d vec = q.vec();
    v->set_x(vec(0));
    v->set_y(vec(1));
    v->set_z(vec(2));
}

void AHRS::UpdateMagnetometer(const std::string message) {
     _mag.ParseFromString(message);
}

void AHRS::UpdateAccelerometer(const std::string message) {
    _accel.ParseFromString(message);
}

void AHRS::UpdateGyroscope(const std::string message) {
    _gyro.ParseFromString(message);
}

void AHRS::UpdateWheel(const std::string message) {
    _wheel.ParseFromString(message);
}

void AHRS::UpdateCommandedVelocity(const std::string message) {
    _vel.ParseFromString(message);
}

void AHRS::Calculate() {
    if(_stepCount++ <= 1) return;
    SetMessageVector();
}

void AHRS::Calibrate(std::shared_ptr<Sensor::ISensor<Eigen::Vector3d>> accelerometer, std::shared_ptr<Sensor::ISensor<Eigen::Vector3d>> magnetometer) {
    const int sampleSize = 20;
    Eigen::Vector3d m (0,0,0), a (0,0,0);
    for (int i=0; i<sampleSize; i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
        m += magnetometer->GetReading();
        a += accelerometer->GetReading();
    }
    m /= sampleSize;
    _calibratedMagnetometer.w() = 0;
    _calibratedMagnetometer.vec() << m(0), m(1), m(2);
    a /= sampleSize;
    Eigen::Vector3d east = a.cross(m);
    Eigen::Vector3d north = east.cross(a);
    a.normalize();
    east.normalize();
    north.normalize();
    Eigen::Matrix3d mx;
    mx << north(0), north(1), north(2),
          east(0), east(1), east(2),
          -a(0), -a(1), -a(2);
    _orientation.w() = 0;
    _orientation.vec() << 1, 0 ,0;
    _orientation = _orientation * mx;
}

void AHRS::Publish() {
    Calculate();
    _messageNode->Publish(_ahrsTopic, _ahrs);
}
};
