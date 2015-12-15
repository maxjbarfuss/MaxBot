#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>

#include <Dense>
#include <Vector3.pb.h>
#include <MessageBroker.h>

#include <AHRS/AHRS.hpp>
#include <IO/I2CFactory.hpp>
#include <IO/WiringPi/WiringPiI2C.hpp>
#include <Publishers/RangeSensorPublisher.hpp>
#include <Publishers/VectorSensorPublisher.hpp>
#include <Sensor/ISensor.h>
#include <Sensor/IMU/ADXL345.hpp>
#include <Sensor/IMU/L3G4200D.hpp>
#include <Sensor/IMU/HMC5883L.hpp>
#include <Sensor/Range/HCSR04.hpp>

int main()
{
    IO::I2CFactory i2CFactory (std::unique_ptr<IO::I2CCreatorBase>(new IO::I2CCreator<IO::WiringPiI2C>));
    auto maxBotNode = std::make_shared<MaxBotMessages::MessageBroker>("AHRS",1);
    auto accelerometer = std::make_shared<Sensor::ADXL345>(i2CFactory);
    auto gyroscope = std::make_shared<Sensor::L3G4200D>(i2CFactory);
    auto magnetometer = std::make_shared<Sensor::HMC5883L>(i2CFactory);
    auto ahrs = std::make_shared<AHRS::AHRS>(maxBotNode);
    auto frontRightSonic = std::make_shared<Sensor::HCSR04>(1,4);
    auto frontLeftSonic = std::make_shared<Sensor::HCSR04>(5,6);
    auto rearLeftSonic = std::make_shared<Sensor::HCSR04>(28,29);
    auto rearRightSonic = std::make_shared<Sensor::HCSR04>(26,27);

    //accelerometer->Calibrate();
    //gyroscope->Calibrate();
    //magnetometer->Calibrate();
    //ahrs->Calibrate(accelerometer, magnetometer);

    Publishers::VectorSensorPublisher gyroscopePublisher (maxBotNode, "GYRO", "IMU1", gyroscope);
    Publishers::VectorSensorPublisher accelerometerPublisher (maxBotNode, "ACEL", "IMU1", accelerometer);
    Publishers::VectorSensorPublisher magnetometerPublisher (maxBotNode, "MAGN", "IMU1", magnetometer);
    Publishers::RangeSensorPublisher frontRightSonicPublisher (maxBotNode, "RANG", "FRS", frontRightSonic);
    Publishers::RangeSensorPublisher frontLeftSonicPublisher (maxBotNode, "RANG", "FLS", frontLeftSonic);
    Publishers::RangeSensorPublisher rearLeftSonicPublisher (maxBotNode, "RANG", "RLS", rearLeftSonic);
    Publishers::RangeSensorPublisher rearRightSonicPublisher (maxBotNode, "RANG", "RRS", rearRightSonic);

    while(true)
    {
//        std::this_thread::sleep_for(std::chrono::milliseconds(40));
        gyroscopePublisher.Publish();
        accelerometerPublisher.Publish();
        magnetometerPublisher.Publish();
        maxBotNode->DoWork();
        ahrs->Publish();
        //frontRightSonicPublisher.Publish();
        //frontLeftSonicPublisher.Publish();
        //rearLeftSonicPublisher.Publish();
        rearRightSonicPublisher.Publish();

    }
}

