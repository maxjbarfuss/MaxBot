#pragma once

#include <Dense>

namespace Computation {

class ProbabalisticAverageFilter {
private:
    Eigen::Vector3d _averageAngle;
    double _accuracyTotal;
    unsigned _count;
public:
    ProbabalisticAverageFilter() : _accuracyTotal(0), _count(0) {
        _averageAngle = Eigen::Vector3d(0,0,0);
    }

    virtual void AddValue(Eigen::Vector3d const &angle, double accuracy) {
        _accuracyTotal += accuracy;
        if (_count++ == 0) {
            _averageAngle = angle;
        } else {
            auto averageAccuracy = _accuracyTotal / _count;
            auto accuracySum = averageAccuracy + accuracy;
            Eigen::Quaterniond q1;
            q1.w() = 0;
            q1.vec() = _averageAngle;
            Eigen::Quaterniond q2;
            q2.w() = 0;
            q2.vec() = angle;
            q1 = q1.slerp((accuracy / accuracySum / _count), q2);
            _averageAngle = q1.vec();
        }
    }

    virtual void Clear() {
        _count = 0;
        _accuracyTotal = 0;
        _averageAngle = Eigen::Vector3d(0,0,0);
    }

    virtual Eigen::Vector3d GetFilteredValue() {
        return _averageAngle;
    }
};

};
