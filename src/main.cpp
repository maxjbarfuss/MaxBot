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
#include <Sensor/Range/GP2Y0A21YK.hpp>
#include <Sensor/Range/HCSR04.hpp>
#include <Computation/IFilter.h>

int main()
{
    IO::I2CFactory i2CFactory (std::unique_ptr<IO::I2CCreatorBase>(new IO::I2CCreator<IO::WiringPiI2C>));
    Computation::FilterFactory filterFactory;
    auto maxBotNode = std::make_shared<MaxBotMessages::MessageBroker>("AHRS", 1);
    auto accelerometer = std::make_shared<Sensor::ADXL345>(i2CFactory);
    auto gyroscope = std::make_shared<Sensor::L3G4200D>(i2CFactory);
    auto magnetometer = std::make_shared<Sensor::HMC5883L>(i2CFactory);
    auto ahrs = std::make_shared<AHRS::AHRS>(maxBotNode);
    auto frontRightSonic = std::make_shared<Sensor::HCSR04>(1, 4, filterFactory);
    auto frontLeftSonic = std::make_shared<Sensor::HCSR04>(5, 6, filterFactory);
    auto rearLeftSonic = std::make_shared<Sensor::HCSR04>(28, 29, filterFactory);
    auto rearRightSonic = std::make_shared<Sensor::HCSR04>(26, 27, filterFactory);
    auto frontDownIR = std::make_shared<Sensor::GP2Y0A21YK>(0, 0, filterFactory);

    //accelerometer->Calibrate();
    //gyroscope->Calibrate();
    //magnetometer->Calibrate();
    //ahrs->Calibrate(accelerometer, magnetometer);

    Publishers::VectorSensorPublisher gyroscopePublisher (maxBotNode, "GYRO", "IMU1", gyroscope);
    Publishers::VectorSensorPublisher accelerometerPublisher (maxBotNode, "ACEL", "IMU1", accelerometer);
    Publishers::VectorSensorPublisher magnetometerPublisher (maxBotNode, "MAGN", "IMU1", magnetometer);
    Publishers::RangeSensorPublisher frontRightSonicPublisher (maxBotNode, "RANG", "FRS1", frontRightSonic);
    Publishers::RangeSensorPublisher frontLeftSonicPublisher (maxBotNode, "RANG", "FLS1", frontLeftSonic);
    Publishers::RangeSensorPublisher rearLeftSonicPublisher (maxBotNode, "RANG", "RLS1", rearLeftSonic);
    Publishers::RangeSensorPublisher rearRightSonicPublisher (maxBotNode, "RANG", "RRS1", rearRightSonic);
    Publishers::RangeSensorPublisher frontDownIRPublisher (maxBotNode, "RANG", "FIR1", frontDownIR);

    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
        gyroscopePublisher.Publish();
        accelerometerPublisher.Publish();
        magnetometerPublisher.Publish();
        maxBotNode->DoWork();
        ahrs->Publish();
        frontRightSonicPublisher.Publish();
        frontLeftSonicPublisher.Publish();
        rearLeftSonicPublisher.Publish();
        rearRightSonicPublisher.Publish();
        frontIRPublisher.Publish();
    }
}

