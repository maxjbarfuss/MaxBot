#pragma once

#include <tuple>
#include <cmath>
#include <thread>
#include <future>
#include <chrono>

#include <MotionControl/RoboClaw.h>

namespace Motion {

#define Kp              3
#define Ki              .25
#define Kd              .2
#define QPPSR1          2320            //max quadrature pulses per second
#define QPPSR2          2382            //max quadrature pulses per second
#define QPPSF1          2317            //max quadrature pulses per second
#define QPPSF2          2229            //max quadrature pulses per second
#define FRONT_DEVICE    "/dev/ttyAMA0"
#define REAR_DEVICE     "/dev/ttyUSB0"
#define ADDRESS         0x80
#define QPR             736.5           //quadurature pulses per rotation
#define STEP_TIME       .06             //seconds
#define ACCELERATION    4096            //quadrature pulses per second per second
#define BAUD            38400
#define CIRCUMFERENCE   .3444           //meters
#define WHEEL_BASE      .25             //meters
#define AXEL_DISTANCE   .23             //meters

class VelocityControl {
private:
    RoboClaw                                                        _rear;
    RoboClaw                                                        _front;
    int                                                             _leftSpeed;
    int                                                             _rightSpeed;
    std::chrono::time_point<std::chrono::steady_clock>              _rearTime;
    std::chrono::time_point<std::chrono::steady_clock>              _frontTime;
    int                                                             _lastRearLeft;
    int                                                             _lastRearRight;
    int                                                             _lastFrontLeft;
    int                                                             _lastFrontRight;

    static int DesiredDistance(int currentSpeed, int desiredSpeed) {
        auto timeToDesiredSpeed = (desiredSpeed - currentSpeed) / ACCELERATION;
        if (timeToDesiredSpeed > STEP_TIME)
            return std::round((currentSpeed * STEP_TIME) + (ACCELERATION * STEP_TIME * STEP_TIME) / 2.0);
        auto distanceToDesiredSpeed = currentSpeed * timeToDesiredSpeed + (ACCELERATION * timeToDesiredSpeed * timeToDesiredSpeed) / 2.0;
        auto timeAtSpeed = STEP_TIME - timeToDesiredSpeed;
        auto distanceAtSpeed = desiredSpeed * timeAtSpeed;
        return std::round(distanceToDesiredSpeed + distanceAtSpeed);
    }

    std::tuple<int, int> RunMotors(RoboClaw& motor, int address, double leftSpeed, double rightSpeed, std::chrono::time_point<std::chrono::steady_clock>& lastTime) {
        uint8_t status;
        bool valid;
        auto leftDistance = DesiredDistance(motor.ReadSpeedM1(address, &status, &valid), leftSpeed);
        auto rightDistance = DesiredDistance(motor.ReadSpeedM2(address, &status, &valid), rightSpeed);
        motor.SpeedAccelDistanceM1M2(address, ACCELERATION, leftSpeed, std::abs(leftDistance), rightSpeed, std::abs(rightDistance), 1);
        motor.SpeedAccelDistanceM1M2(address, ACCELERATION, 0, std::abs(leftDistance), 0, std::abs(rightDistance), 0);
        auto now = std::chrono::steady_clock::now();
        auto timeSinceLastTime = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(STEP_TIME * 1000) - timeSinceLastTime));
        leftDistance = motor.ReadEncM1(address, &status, &valid);
        rightDistance = motor.ReadEncM2(address, &status, &valid);
        lastTime = std::chrono::steady_clock::now();
        return std::make_tuple(leftDistance, rightDistance);
    }

public:
    VelocityControl()
        : _rear(REAR_DEVICE, BAUD, true), _front(FRONT_DEVICE, BAUD, true), _leftSpeed(0), _rightSpeed(0),
        _lastRearLeft(0), _lastRearRight(0), _lastFrontLeft(0), _lastFrontRight(0) {
        _rear.SetM1VelocityPID(ADDRESS, Kp, Ki, Kd, QPPSR1);
        _rear.SetM2VelocityPID(ADDRESS, Kp, Ki, Kd, QPPSR2);
        _front.SetM1VelocityPID(ADDRESS, Kp, Ki, Kd, QPPSF1);
        _front.SetM2VelocityPID(ADDRESS, Kp, Ki, Kd, QPPSF2);
        _rear.ResetEncoders(ADDRESS);
        _front.ResetEncoders(ADDRESS);
        _rearTime = std::chrono::steady_clock::now();
        _frontTime = std::chrono::steady_clock::now();
    }

    /// ***********************************************
    /// SetVelocity in linear m/s and angular rad/s
    /// ***********************************************
    void SetVelocity(double linear, double angular) {
        double angularSpeed = (angular * WHEEL_BASE) / 2;
        _leftSpeed = std::round((linear - angularSpeed) * QPR / CIRCUMFERENCE);
        _rightSpeed = std::round((linear + angularSpeed) * QPR / CIRCUMFERENCE);
    }

    static void WaitForFuture(std::future<std::tuple<int, int>>& future, bool& success, int& left, int& right) {
        auto status = future.wait_for(std::chrono::milliseconds(static_cast<int>(STEP_TIME * 1200)));
        if (status == std::future_status::ready) {
            success = true;
            auto v = future.get();
            left = std::get<0>(v);
            right = std::get<1>(v);
        }
    }

    /// ************************************************
    /// RunMotors sets x, y and heading delta since last
    /// call to RunMotors
    /// ************************************************
    void RunMotors(double& x, double& y, double& heading) {
        int frontLeft, frontRight, rearLeft, rearRight;
        bool frontSuccess = false;
        bool rearSuccess = false;
        std::chrono::time_point<std::chrono::steady_clock> lastRearTime = _rearTime;
        std::chrono::time_point<std::chrono::steady_clock> lastFrontTime = _frontTime;
        std::future<std::tuple<int, int>> rearFuture = std::async(std::launch::async, [&](){
            return RunMotors(_rear, ADDRESS, _leftSpeed, _rightSpeed, _rearTime);
        });
        std::future<std::tuple<int, int>> frontFuture = std::async(std::launch::async, [&](){
            return RunMotors(_front, ADDRESS, _leftSpeed, _rightSpeed, _frontTime);
        });
        std::thread rearThread ([&]{ WaitForFuture(rearFuture, rearSuccess, rearLeft, rearRight); });
        std::thread frontThread ([&]{ WaitForFuture(frontFuture, frontSuccess, frontLeft, frontRight); });
        rearThread.join();
        frontThread.join();

        double time = 0;
        double Vl = 0;
        double Vr = 0;
        time += (rearSuccess) ? std::chrono::duration_cast<std::chrono::milliseconds>(_rearTime - lastRearTime).count() / 1000.0 : 0;
        time += (frontSuccess) ? std::chrono::duration_cast<std::chrono::milliseconds>(_frontTime - lastFrontTime).count() / 1000.0 : 0;
        time /= (frontSuccess && rearSuccess) ? 2.0 : 1.0;
        if (time > (STEP_TIME / 2) && time < (STEP_TIME * 1.5)) {
            Vl += (rearSuccess) ? (rearLeft -_lastRearLeft) : 0;
            Vl += (frontSuccess) ? (frontLeft -_lastFrontLeft) : 0;
            Vl /= (frontSuccess && rearSuccess) ? 2.0 : 1.0;
            Vr += (rearSuccess) ? (rearRight -_lastRearRight) : 0;
            Vr += (frontSuccess) ? (frontRight -_lastFrontRight) : 0;
            Vr /= (frontSuccess && rearSuccess) ? 2.0 : 1.0;
            Vl = ((Vl / QPR)  * CIRCUMFERENCE) / time;
            Vr = ((Vr / QPR)  * CIRCUMFERENCE) / time;
        }
        double theta = (Vr - Vl) / WHEEL_BASE;
        _lastRearLeft = rearLeft;
        _lastRearRight = rearRight;
        _lastFrontLeft = frontLeft;
        _lastFrontRight = frontRight;

        x = (((Vr + Vl) * cos(theta)) / 2.0) * time;
        y = (((Vr + Vl) * sin(theta)) / 2.0) * time;
        heading = theta * time;
    }
};

};
