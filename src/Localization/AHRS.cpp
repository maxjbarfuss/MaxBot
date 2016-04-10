#include <limits>
#include <algorithm>

#include <Localization/AHRS.h>
#include <iostream>

namespace Localization {

#define MICROSEC_PER_SEC 1000000.0

AHRS::AHRS(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, const std::string ahrsTopic)
: _messageNode(messageNode), _stepCount(0), _ahrsTopic(ahrsTopic), _lastCalculatedTime(0), _calcCount(0) {
    _ahrs.mutable_stamp()->set_component_id("AHRS");
    _orientation.w() = 1;
    _orientation.x() = 0;
    _orientation.y() = 0;
    _orientation.z() = 0;
}

Eigen::Vector3d AHRS::GetOrientation() {
    Eigen::Vector3d ret;
    double sqw = _orientation.w() * _orientation.w();
    double sqx = _orientation.x() * _orientation.x();
    double sqy = _orientation.y() * _orientation.y();
    double sqz = _orientation.z() * _orientation.z();
    ret(0) = atan2(2.0 * (_orientation.x() * _orientation.y() + _orientation.z() * _orientation.w()), (sqx - sqy - sqz + sqw));
    ret(1) = asin(-2.0 * (_orientation.x() * _orientation.z() - _orientation.y() * _orientation.w()) / (sqx + sqy + sqz + sqw));
    ret(2) = atan2(2.0 * (_orientation.y() * _orientation.z() + _orientation.x() * _orientation.w()), (-sqx - sqy + sqz + sqw));
    return ret;
}

void AHRS::Calculate(long long t) {
    Eigen::Quaterniond q = _orientation;
    std::lock_guard<std::mutex> lock(_updateMutex);
    auto g = _angularRateFilter.GetFilteredValue();
    if(_lastCalculatedTime > 0) {
        Eigen::Vector4d qDot;
        qDot(0) = (-q.x() * g.x() - q.y() * g.y() - q.z() * g.z()) / 2;
        qDot(1) = ( q.w() * g.x() + q.y() * g.z() - q.z() * g.y()) / 2;
        qDot(2) = ( q.w() * g.y() - q.x() * g.z() + q.z() * g.x()) / 2;
        qDot(3) = ( q.w() * g.z() + q.x() * g.y() - q.y() * g.x()) / 2;
        qDot *= ((t - _lastCalculatedTime) / MICROSEC_PER_SEC);
        q.w() += qDot(0);
        q.x() += qDot(1);
        q.y() += qDot(2);
        q.z() += qDot(3);
        q.normalize();
    }
    _orientation = q;
    _lastCalculatedTime = t;
    _angularRateFilter.Clear();
}

void AHRS::UpdateAngularRate(const Eigen::Vector3d angle, const long long measurementDuration, const long long measurementTime, double accuracy) {
    std::lock_guard<std::mutex> lock(_updateMutex);
    //_angularRateFilter.AddValue(Eigen::Vector3d(angle[0], angle[1], angle[2]), accuracy);
    Eigen::Quaterniond q = _orientation;
    if(_lastCalculatedTime > 0 && measurementTime > _lastCalculatedTime) {
        Eigen::Vector4d qDot;
        qDot(0) = (-q.x() * angle.x() - q.y() * angle.y() - q.z() * angle.z()) / 2;
        qDot(1) = ( q.w() * angle.x() + q.y() * angle.z() - q.z() * angle.y()) / 2;
        qDot(2) = ( q.w() * angle.y() - q.x() * angle.z() + q.z() * angle.x()) / 2;
        qDot(3) = ( q.w() * angle.z() + q.x() * angle.y() - q.y() * angle.x()) / 2;
        qDot *= ((measurementTime - _lastCalculatedTime) / MICROSEC_PER_SEC);
        q.w() += qDot(0);
        q.x() += qDot(1);
        q.y() += qDot(2);
        q.z() += qDot(3);
        q.normalize();
    }
    _orientation = q;
    _lastCalculatedTime = measurementTime;
}

void AHRS::UpdateOrientation(const Eigen::Quaterniond orientation, const long long measurementTime, const double accuracy) {
    std::lock_guard<std::mutex> lock(_updateMutex);
    _orientation = _orientation.slerp(accuracy, orientation);
}

void AHRS::Calibrate(std::shared_ptr<Sensor::ISensor<Eigen::Vector3d>> accelerometer, std::shared_ptr<Sensor::ISensor<Eigen::Vector3d>> magnetometer) {
//    const int sampleSize = 20;
//    Eigen::Vector3d m (0,0,0), a (0,0,0);
//    for (int i=0; i<sampleSize; i++)
//    {
//        std::this_thread::sleep_for(std::chrono::milliseconds(40));
//        m += magnetometer->GetReading();
//        a += accelerometer->GetReading();
//    }
//    m /= sampleSize;
//    //_calibratedMagnetometer.w() = 0;
//    //_calibratedMagnetometer.vec() << m(0), m(1), m(2);
//    a /= sampleSize;
//    Eigen::Vector3d east = a.cross(m);
//    Eigen::Vector3d north = east.cross(a);
//    a.normalize();
//    east.normalize();
//    north.normalize();
//    Eigen::Matrix3d mx;
//    mx << north(0), north(1), north(2),
//          east(0), east(1), east(2),
//          -a(0), -a(1), -a(2);
//    _orientation.w() = 0;
//    _orientation.vec() << 1, 0 ,0;
//    _orientation = _orientation * mx;
}

void AHRS::Publish() {
    auto t = _messageNode->MicrosecondsSinceEpoch();
    //Calculate(t);
    //auto v1 = GetOrientation();
    auto v2 = _ahrs.mutable_quaternion();
    v2->set_w(_orientation.w());
    v2->set_x(_orientation.x());
    v2->set_y(_orientation.y());
    v2->set_z(_orientation.z());
    _ahrs.mutable_stamp()->set_microseconds_since_epoch(t);
    _messageNode->Publish(_ahrsTopic, _ahrs);
}

};
