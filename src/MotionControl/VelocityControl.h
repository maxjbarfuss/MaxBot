#pragma once

#include <tuple>
#include <cmath>
#include <thread>
#include <future>
#include <chrono>
#include <iostream>

#include <MotionControl/IRoboClaw.h>
#include <MotionControl/IRoboClaw.h>

namespace MotionControl {

#define STEP_TIME               .06             //seconds
#define ACCELERATION            1000            //quadrature pulses per second per second

class VelocityControl {
private:
    std::shared_ptr<IRoboClaw>                                      _rear;
    std::shared_ptr<IRoboClaw>                                      _front;
    int                                                             _rearAddress;
    int                                                             _frontAddress;
    double                                                          _wheelCircumference;
    double                                                          _wheelBase;
    double                                                          _qpr;
    int                                                             _leftSpeed;
    int                                                             _rightSpeed;
    std::chrono::time_point<std::chrono::steady_clock>              _rearTime;
    std::chrono::time_point<std::chrono::steady_clock>              _frontTime;
    int                                                             _lastRearLeft;
    int                                                             _lastRearRight;
    int                                                             _lastFrontLeft;
    int                                                             _lastFrontRight;
private:
    static int DesiredDistance(int currentSpeed, int desiredSpeed);
    std::tuple<int, int> RunMotors(std::shared_ptr<IRoboClaw> motor, int address, double leftSpeed, double rightSpeed, std::chrono::time_point<std::chrono::steady_clock>& lastTime);
    static void WaitForFuture(std::future<std::tuple<int, int>>& future, bool& success, int& left, int& right);
public:
    VelocityControl(std::shared_ptr<IRoboClaw> rear, std::shared_ptr<IRoboClaw> front, int rearAddress, int frontAddress, double wheelCircumference, double wheelBase, double qpr);
    /// ***********************************************
    /// SetVelocity in linear m/s and angular rad/s
    /// ***********************************************
    void SetVelocity(double linear, double angular);
    /// ************************************************
    /// RunMotors sets x, y and heading delta since last
    /// call to RunMotors
    /// ************************************************
    void RunMotors(double& x, double& y, double& heading);
};

};
