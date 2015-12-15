#pragma once

#include <cmath>

#include <Dense>
#include <MessageBroker.h>
#include <Sensor/ISensor.h>

namespace AHRS {

#define MAGNETOMETER_TOPIC      "MAGN"
#define GYROSCOPE_TOPIC         "GYRO"
#define ACCELEROMETER_TOPIC     "ACEL"
#define AHRS_TOPIC              "AHRS"

class AHRS {
private:
    MaxBotMessages::Vector3Stamped                  _v;
    std::shared_ptr<MaxBotMessages::MessageBroker>  _messageNode;
    Eigen::Vector3d                                 _ahrs;
    double                                          _stepCount;
    Eigen::Vector3d                                 _lastMagnetometer;
    Eigen::Vector3d                                 _magnetometerAnglularDiff;
    Eigen::Vector3d                                 _gyroscopeAngularDiff;
    int                                             _lastGyrosopeTime;

    void SetMessageVector()
    {
        _v.mutable_stamp()->set_milliseconds_since_epoch(_messageNode->MillisecondsSinceEpoch());
        auto v = _v.mutable_vector();
        v->set_x(_ahrs(0));
        v->set_y(_ahrs(1));
        v->set_z(_ahrs(2));
    }

    static double AngleBetween(double v1x, double v1y, double v2x, double v2y) {
        double d = atan2(v2y,v2x) - atan2(v1y,v1x);
        if (std::isnan(d)) return 0;
        else return(d);
    }

    void UpdateMagnetometer(const std::string message) {
        _v.ParseFromString(message);
        auto v = _v.vector();
        Eigen::Vector3d thisMagnetometer (v.x(), v.y(), v.z());
        _magnetometerAnglularDiff << AngleBetween(v.y(), v.z(), _lastMagnetometer(1), _lastMagnetometer(2)),
                                     AngleBetween(v.x(), v.z(), _lastMagnetometer(0), _lastMagnetometer(2)),
                                     AngleBetween(v.x(), v.y(), _lastMagnetometer(0), _lastMagnetometer(1));
        _lastMagnetometer = thisMagnetometer;
    }

    void UpdateAccelerometer(const std::string message) {
    }

    void UpdateGyroscope(const std::string message) {
        _v.ParseFromString(message);
        auto v = _v.vector();
        Eigen::Vector3d thisGyroscope(v.x(), v.y(), v.z());
        int thisTime = _v.stamp().milliseconds_since_epoch();
        _gyroscopeAngularDiff = thisGyroscope * (_lastGyrosopeTime - thisTime) / 1000;
        _lastGyrosopeTime = thisTime;
    }

    void Calculate() {
        if(_stepCount++ <= 1) return;
        _ahrs += (_magnetometerAnglularDiff + _gyroscopeAngularDiff) / 2;
        SetMessageVector();
    }

public:
    AHRS(std::shared_ptr<MaxBotMessages::MessageBroker> messageNode)
        : _messageNode(messageNode), _stepCount(0), _lastMagnetometer(0,0,0), _magnetometerAnglularDiff(0,0,0), _gyroscopeAngularDiff(0,0,0) {
        _messageNode->Subscribe(MAGNETOMETER_TOPIC, [&](std::string s){ UpdateMagnetometer(s); });
        _messageNode->Subscribe(ACCELEROMETER_TOPIC, [&](std::string s){ UpdateAccelerometer(s); });
        _messageNode->Subscribe(GYROSCOPE_TOPIC, [&](std::string s){ UpdateGyroscope(s); });
    }

    void Calibrate(std::shared_ptr<Sensor::ISensor<Eigen::Vector3d>> accelerometer, std::shared_ptr<Sensor::ISensor<Eigen::Vector3d>> magnetometer) {
        const int sampleSize = 20;
        Eigen::Vector3d m (0,0,0), a (0,0,0);
        for (int i=0; i<sampleSize; i++)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
            m += magnetometer->GetReading();
            a += accelerometer->GetReading();
        }
        m /= sampleSize;
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
        _ahrs << 0,0,0;
    }

    void Publish() {
        Calculate();
        _messageNode->Publish(AHRS_TOPIC, _v);
    }

    Eigen::Vector3d GetReading()
    {
        Calculate();
        return _ahrs;
    }
};

};
