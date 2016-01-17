#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>

#include <Dense>
#include <Vector.pb.h>
#include <MessageBroker.h>

#include <AHRS/AHRS.hpp>
#include <Computation/IFilter.h>
#include <IO/I2CFactory.hpp>
#include <IO/WiringPi/WiringPiI2C.hpp>
#include <IO/WiringPi/WiringPiSPI.hpp>
#include <MotionControl/IRoboClaw.h>
#include <MotionControl/RoboClawV4.h>
#include <MotionControl/RoboClawV5.h>
#include <MotionControl/VelocityControl.h>
#include <Publishers/MotionPublisher.h>
#include <Publishers/RangeSensorPublisher.h>
#include <Publishers/VectorSensorPublisher.h>
#include <Sensor/ISensor.h>
#include <Sensor/IMU/ADXL345.h>
#include <Sensor/IMU/L3G4200D.h>
#include <Sensor/IMU/HMC5883L.h>
#include <Sensor/Range/GP2Y0A02YK.h>
#include <Sensor/Range/GP2Y0A21YK.h>
#include <Sensor/Range/GP2Y0A710K.h>
#include <Sensor/Range/HCSR04.h>
#include <Subscribers/TeleopSubscriber.h>

/// RoboClaw Settings
#define KpV5                    2500.0
#define KiV5                    600.0
#define KdV5                    32767.0
#define KpV4                    2500.0
#define KiV4                    100.0
#define KdV4                    200.0
#define QPPS                    2250            //max quadrature pulses per second
#define FRONT_DEVICE            "/dev/ttyAMA0"
#define REAR_DEVICE             "/dev/ttyACM0"
#define FRONT_BAUD              38400
#define REAR_BAUD               115200
#define FRONT_ADDRESS           0x80
#define REAR_ADDRESS            0x80
///Platform settings
#define QPR                     736.5           //quadurature pulses per rotation
#define WHEEL_CIRCUMFERENCE     .20             //meters
#define WHEEL_BASE              .25             //meters
#define AXEL_DISTANCE           .23             //meters

int main()
{
    IO::I2CFactory i2CFactory (std::unique_ptr<IO::I2CCreatorBase>(new IO::I2CCreator<IO::WiringPiI2C>));
//    auto spi = std::make_shared<IO::WiringPiSPI>(64, 0);
//    Computation::FilterFactory filterFactory;
//    auto spiNode = std::make_shared<MaxBotMessages::MessageBroker>(1);
//    auto gpioNode = std::make_shared<MaxBotMessages::MessageBroker>(1);
    auto motionNode = std::make_shared<MaxBotMessages::MessageBroker>(1);
    auto imuNode = std::make_shared<MaxBotMessages::MessageBroker>(1);
    auto accelerometer = std::make_shared<Sensor::ADXL345>(i2CFactory, Sensor::ADXL345_DATARATE_50_HZ, Sensor::ADXL345_RANGE_16_G);
    auto gyroscope = std::make_shared<Sensor::L3G4200D>(i2CFactory, L3G4200D_REG_SCALE2000);
    auto magnetometer = std::make_shared<Sensor::HMC5883L>(i2CFactory, HMC5883L_SCALE_1_3, HMC5883L_MODE_SINGLE);
//    auto ahrs = std::make_shared<AHRS::AHRS>(imuNode);
//    auto frontLeftSonic = std::make_shared<Sensor::HCSR04>(5, 6, filterFactory);
//    auto frontRightSonic = std::make_shared<Sensor::HCSR04>(1, 4, filterFactory);
//    auto frontLeftLeftSonic = std::make_shared<Sensor::HCSR04>(24, 25, filterFactory);
//    auto frontRightRightSonic = std::make_shared<Sensor::HCSR04>(22, 23, filterFactory);
//    auto rearLeftSonic = std::make_shared<Sensor::HCSR04>(28, 29, filterFactory);
//    auto rearRightSonic = std::make_shared<Sensor::HCSR04>(26, 27, filterFactory);
//    auto frontDownIR = std::make_shared<Sensor::GP2Y0A21YK>(spi, 0, filterFactory);
//    auto rearIR = std::make_shared<Sensor::GP2Y0A21YK>(spi, 2, filterFactory);
//    auto frontIR = std::make_shared<Sensor::GP2Y0A710K>(spi, 1, filterFactory);
//    auto frontLeftIR = std::make_shared<Sensor::GP2Y0A02YK>(spi,4 , filterFactory);
//    auto frontRightIR = std::make_shared<Sensor::GP2Y0A02YK>(spi, 5, filterFactory);
    auto rear = std::make_shared<MotionControl::RoboClawV5>(REAR_DEVICE, REAR_BAUD);
    rear->SetM1VelocityPID(REAR_ADDRESS, KpV5, KiV5, KdV5, QPPS);
    rear->SetM2VelocityPID(REAR_ADDRESS, KpV5, KiV5, KdV5, QPPS);
    rear->ResetEncoders(REAR_ADDRESS);
    auto front = std::make_shared<MotionControl::RoboClawV4>(FRONT_DEVICE, FRONT_BAUD, true);
    front->SetM1VelocityPID(FRONT_ADDRESS, KpV4, KiV4, KdV4, QPPS);
    front->SetM2VelocityPID(FRONT_ADDRESS, KpV4, KiV4, KdV4, QPPS);
    front->ResetEncoders(FRONT_ADDRESS);
    auto velocityControl = std::make_shared<MotionControl::VelocityControl>(rear, front, REAR_ADDRESS, FRONT_ADDRESS, WHEEL_CIRCUMFERENCE, WHEEL_BASE, QPR);

    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(15,25);
    accelerometer->StartCalibration();
    gyroscope->StartCalibration();
    magnetometer->StartCalibration();
    for (int i = 1; i < 128; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(distribution(generator)));
        accelerometer->StepCalibration(i);
        gyroscope->StepCalibration(i);
        magnetometer->StepCalibration(i);
    }
    accelerometer->EndCalibration();
    gyroscope->EndCalibration();
    magnetometer->EndCalibration();

    Publishers::VectorSensorPublisher gyroscopePublisher (imuNode, "GYRO", "IMU1", gyroscope);
    Publishers::VectorSensorPublisher accelerometerPublisher (imuNode, "ACEL", "IMU1", accelerometer);
    Publishers::VectorSensorPublisher magnetometerPublisher (imuNode, "MAGN", "IMU1", magnetometer);
//    Publishers::RangeSensorPublisher frontRightSonicPublisher (gpioNode, "RANG", "FRS1", frontRightSonic);
//    Publishers::RangeSensorPublisher frontLeftSonicPublisher (gpioNode, "RANG", "FLS1", frontLeftSonic);
//    Publishers::RangeSensorPublisher frontRightRightSonicPublisher (gpioNode, "RANG", "FRS2", frontRightRightSonic);
//    Publishers::RangeSensorPublisher frontLeftLeftSonicPublisher (gpioNode, "RANG", "FLS2", frontLeftLeftSonic);
//    Publishers::RangeSensorPublisher rearLeftSonicPublisher (gpioNode, "RANG", "RLS1", rearLeftSonic);
//    Publishers::RangeSensorPublisher rearRightSonicPublisher (gpioNode, "RANG", "RRS1", rearRightSonic);
//    Publishers::RangeSensorPublisher frontDownIRPublisher (spiNode, "RANG", "FIR1", frontDownIR);
//    Publishers::RangeSensorPublisher rearIRPublisher (spiNode, "RANG", "RIR1", rearIR);
//    Publishers::RangeSensorPublisher frontIRPublisher (spiNode, "RANG", "FIR2", frontIR);
//    Publishers::RangeSensorPublisher frontLeftIRPublisher (spiNode, "RANG", "FIR3", frontLeftIR);
//    Publishers::RangeSensorPublisher frontRightIRPublisher (spiNode, "RANG", "FIR4", frontRightIR);
    Publishers::MotionPublisher motionPublisher (motionNode, "POS2", "VC1", velocityControl);
    Subscribers::TeleopSubscriber teleop (motionNode, "TELE", velocityControl);

    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        gyroscopePublisher.Publish();
        accelerometerPublisher.Publish();
        magnetometerPublisher.Publish();
//        ahrs->Publish();
//        frontRightSonicPublisher.Publish();
//        frontLeftSonicPublisher.Publish();
//        frontRightRightSonicPublisher.Publish();
//        frontLeftLeftSonicPublisher.Publish();
//        rearLeftSonicPublisher.Publish();
//        rearRightSonicPublisher.Publish();
//        frontDownIRPublisher.Publish();
//        rearIRPublisher.Publish();
//        frontIRPublisher.Publish();
//        frontLeftIRPublisher.Publish();
//        frontRightIRPublisher.Publish();
//        //velocityControl->SetVelocity(v+=.001, 0);
        motionPublisher.Publish();
        motionNode->DoWork();
        imuNode->DoWork();
    }
}

