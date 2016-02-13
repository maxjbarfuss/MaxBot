#include <MotionControl/VelocityControl.h>
#include <iomanip>

namespace MotionControl {

VelocityControl::VelocityControl(std::shared_ptr<IRoboClaw> rear, std::shared_ptr<IRoboClaw> front, int rearAddress, int frontAddress, double wheelCircumference, double wheelBase, double qpr)
    :  _rear(rear), _front(front), _rearAddress(rearAddress), _frontAddress(frontAddress), _wheelCircumference(wheelCircumference), _wheelBase(wheelBase), _qpr(qpr),
    _leftSpeed(0), _rightSpeed(0), _lastRearLeft(0), _lastRearRight(0), _lastFrontLeft(0), _lastFrontRight(0) {
    _rearTime = std::chrono::steady_clock::now();
    _frontTime = std::chrono::steady_clock::now();
}

int VelocityControl::DesiredDistance(int currentSpeed, int desiredSpeed) {
    auto timeToDesiredSpeed = (desiredSpeed - currentSpeed) / ACCELERATION;
    if (timeToDesiredSpeed > STEP_TIME)
        return std::round((currentSpeed * STEP_TIME) + (ACCELERATION * STEP_TIME * STEP_TIME) / 2.0);
    auto distanceToDesiredSpeed = currentSpeed * timeToDesiredSpeed + (ACCELERATION * timeToDesiredSpeed * timeToDesiredSpeed) / 2.0;
    auto timeAtSpeed = STEP_TIME - timeToDesiredSpeed;
    auto distanceAtSpeed = desiredSpeed * timeAtSpeed;
    return std::round(distanceToDesiredSpeed + distanceAtSpeed);
}

std::tuple<int, int> VelocityControl::RunMotors(std::shared_ptr<IRoboClaw> motor, int address, double leftSpeed, double rightSpeed, std::chrono::time_point<std::chrono::steady_clock>& lastTime) {
    uint8_t status;
    bool valid;
    auto leftDistance = DesiredDistance(motor->ReadSpeedM1(address, &status, &valid), leftSpeed);
    auto rightDistance = DesiredDistance(motor->ReadSpeedM2(address, &status, &valid), rightSpeed);
    motor->SpeedAccelDistanceM1M2(address, ACCELERATION, leftSpeed, std::abs(leftDistance), rightSpeed, std::abs(rightDistance), 1);
    motor->SpeedAccelDistanceM1M2(address, ACCELERATION, 0, std::abs(leftDistance), 0, std::abs(rightDistance), 0);
    auto now = std::chrono::steady_clock::now();
    auto timeSinceLastTime = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count();
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(STEP_TIME * 1000) - timeSinceLastTime));
    leftDistance = motor->ReadEncM1(address, &status, &valid);
    rightDistance = motor->ReadEncM2(address, &status, &valid);
    lastTime = std::chrono::steady_clock::now();
    return std::make_tuple(leftDistance, rightDistance);
}

void VelocityControl::WaitForFuture(std::future<std::tuple<int, int>>& future, bool& success, int& left, int& right) {
    auto status = future.wait_for(std::chrono::milliseconds(static_cast<int>(STEP_TIME * 1200)));
    if (status == std::future_status::ready) {
        success = true;
        auto v = future.get();
        left = std::get<0>(v);
        right = std::get<1>(v);
    }
}

void VelocityControl::SetVelocity(double linear, double angular) {
    double angularSpeed = (angular * _wheelBase) / 2;
    _leftSpeed = std::round((linear - angularSpeed) * _qpr / _wheelCircumference);
    _rightSpeed = std::round((linear + angularSpeed) * _qpr / _wheelCircumference);
}

void VelocityControl::RunMotors(double& x, double& y, double& heading) {
    int frontLeft, frontRight, rearLeft, rearRight;
    bool frontSuccess = false;
    bool rearSuccess = false;
    std::chrono::time_point<std::chrono::steady_clock> lastRearTime = _rearTime;
    std::chrono::time_point<std::chrono::steady_clock> lastFrontTime = _frontTime;
    std::future<std::tuple<int, int>> rearFuture = std::async(std::launch::async, [&](){
        return RunMotors(_rear, _rearAddress, _leftSpeed, _rightSpeed, _rearTime);
    });
    std::future<std::tuple<int, int>> frontFuture = std::async(std::launch::async, [&](){
        return RunMotors(_front, _frontAddress, _leftSpeed, _rightSpeed, _frontTime);
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
        Vl = ((Vl / _qpr)  * _wheelCircumference) / time;
        Vr = ((Vr / _qpr)  * _wheelCircumference) / time;
    }
    double theta = (Vr - Vl) / _wheelBase;
    _lastRearLeft = rearLeft;
    _lastRearRight = rearRight;
    _lastFrontLeft = frontLeft;
    _lastFrontRight = frontRight;

    x = (((Vr + Vl) * cos(theta)) / 2.0) * time;
    y = (((Vr + Vl) * sin(theta)) / 2.0) * time;
    heading = theta * time;
}

};
