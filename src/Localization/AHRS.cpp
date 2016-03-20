#include <limits>
#include <algorithm>

#include <Localization/AHRS.h>

namespace Localization {

AHRS::AHRS(std::shared_ptr<MaxBotMessages::IMessageBroker> messageNode, Eigen::Quaterniond imuOrientation, Eigen::Vector3d imuOffset,
     const std::string magnTopic, const std::string gyroTopic, const std::string accelTopic, const std::string wheelTopic, const std::string cvelTopic, const std::string ahrsTopic)
: _messageNode(messageNode), _stepCount(0), _ahrsTopic(ahrsTopic), _lastMeasurementTime(0), _lastCalculatedTime(0) {
    _ahrs.mutable_stamp()->set_component_id("AHRS");
    Eigen::Vector3d v1 (1,0,0);
    _orientation.w() = 0;
    _orientation.vec() = v1;
    Eigen::Vector3d v2 (0,0,0);
    _angularRate.w() = 0;
    _angularRate.vec() = v2;
}

void AHRS::Calculate() {
    std::lock_guard<std::mutex> lock(_updateMutex);
    if(_lastCalculatedTime > 0 && _lastMeasurementTime > 0) {
        _ahrs.mutable_stamp()->set_microseconds_since_epoch(_messageNode->MicrosecondsSinceEpoch());
        Eigen::Vector3d v1;
        v1 = _angularRate.vec();
        v1 = v1 * ((_lastMeasurementTime - _lastCalculatedTime) / 1000000);
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(v1.x(), Eigen::Vector3d::UnitX())
          * Eigen::AngleAxisd(v1.y(), Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(v1.z(), Eigen::Vector3d::UnitZ());
        _orientation = q * _orientation * q.inverse();
        _orientation.normalize();
        v1 = _orientation.vec();
        auto v2 = _ahrs.mutable_vector();
        v2->set_x(v1.x());
        v2->set_y(v1.y());
        v2->set_z(v1.z());
    }
   _lastCalculatedTime = _messageNode->MicrosecondsSinceEpoch();
   _angularRateAccuracy = 0;
}

void AHRS::UpdateAngularRate(const std::array<double, 3> angle, const long measurementTime, double accuracy) {
    if (measurementTime < _lastCalculatedTime) return;
    std::lock_guard<std::mutex> lock(_updateMutex);
    _lastMeasurementTime = measurementTime;
    Eigen::Vector3d v (angle[0], angle[1], angle[2]);
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(v.x(), Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(v.y(), Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(v.z(), Eigen::Vector3d::UnitZ());
    if (_angularRateAccuracy < std::numeric_limits<double>::min()) {
        _angularRate = q;
    } else if (accuracy >= _angularRateAccuracy) {
        _angularRate = _angularRate.slerp(1 - _angularRateAccuracy / accuracy, q);
    } else {
        _angularRate = _angularRate.slerp(_angularRateAccuracy / accuracy, q);
    }
    _angularRateAccuracy = std::max(_angularRateAccuracy, accuracy);
    _angularRate.normalize();
}

std::array<double, 3> AHRS::UpdateAbsoluteOrientation(const std::array<double, 3> angle, const long measurementTime, const double accuracy) {
    std::lock_guard<std::mutex> lock(_updateMutex);
    return {0,0,0};
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
    Calculate();
    _messageNode->Publish(_ahrsTopic, _ahrs);
}

};
