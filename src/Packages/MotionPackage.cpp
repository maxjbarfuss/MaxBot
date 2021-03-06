#include <Packages/MotionPackage.h>
#include <MotionControl/IRoboClaw.h>
#include <MotionControl/RoboClawV5.h>
#include <MotionControl/VelocityControl.h>
#include <Publishers/MotionPublisher.h>
#include <Publishers/VoltagePublisher.h>
#include <Sensor/MotorController/RoboClawVoltage.h>

namespace Packages {

/// RoboClaw Settings
#define Kp                      2500.0
#define Ki                      600.0
#define Kd                      10000.0
#define QPPS                    2250            //max quadrature pulses per second
#define FRONT_DEVICE            "/dev/ttyACM1"
#define REAR_DEVICE             "/dev/ttyACM0"
#define FRONT_BAUD              115200
#define REAR_BAUD               115200
#define FRONT_ADDRESS           0x80
#define REAR_ADDRESS            0x80
///Platform settings
#define QPR                     736.5           //quadurature pulses per rotation
#define WHEEL_CIRCUMFERENCE     .37             //meters
#define WHEEL_BASE              .25             //meters
#define AXEL_DISTANCE           .23             //meters
#define BODY_WIDTH              .32             //meters
#define BODY_LENGTH             .35             //meters

MotionPackage::MotionPackage() : MaxBotPackageBase() {
    auto rear = std::make_shared<MotionControl::RoboClawV5>(REAR_DEVICE, REAR_BAUD);
    rear->SetM1VelocityPID(REAR_ADDRESS, Kp, Ki, Kd, QPPS);
    rear->SetM2VelocityPID(REAR_ADDRESS, Kp, Ki, Kd, QPPS);
    rear->ResetEncoders(REAR_ADDRESS);
    auto front = std::make_shared<MotionControl::RoboClawV5>(FRONT_DEVICE, FRONT_BAUD);
    front->SetM1VelocityPID(FRONT_ADDRESS, Kp, Ki, Kd, QPPS);
    front->SetM2VelocityPID(FRONT_ADDRESS, Kp, Ki, Kd, QPPS);
    front->ResetEncoders(FRONT_ADDRESS);
    auto velocityControl = std::make_shared<MotionControl::VelocityControl>(rear, front, REAR_ADDRESS, FRONT_ADDRESS, WHEEL_CIRCUMFERENCE, WHEEL_BASE, QPR);
    _publishers.push_back(std::make_tuple(std::make_unique<Publishers::MotionPublisher>(_messageNode, "WHEL", "CVEL", "VC1", velocityControl), std::chrono::microseconds(60000), std::chrono::steady_clock::now()));
    _publishers.push_back(std::make_tuple(std::make_unique<Publishers::VoltagePublisher>(_messageNode, "VOLT", "MC2", std::make_unique<Sensor::RoboClawVoltage>(rear, REAR_ADDRESS)), std::chrono::microseconds(100000), std::chrono::steady_clock::now()));
    _publishers.push_back(std::make_tuple(std::make_unique<Publishers::VoltagePublisher>(_messageNode, "VOLT", "MC1", std::make_unique<Sensor::RoboClawVoltage>(front, FRONT_ADDRESS)), std::chrono::microseconds(100000), std::chrono::steady_clock::now()));
    _teleopSubscriber = std::make_unique<Subscribers::TeleopSubscriber>(_messageNode, "TELE", velocityControl);
}

};
