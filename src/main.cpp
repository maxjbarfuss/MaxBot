#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>

#include <Dense>
#include <Vector.pb.h>
#include <MessageBroker.h>

#include <AHRS/AHRS.hpp>
#include <IO/I2CFactory.hpp>
#include <IO/WiringPi/WiringPiI2C.hpp>
#include <IO/WiringPi/WiringPiSPI.hpp>
#include <MotionControl/VelocityControl.hpp>
#include <Publishers/RangeSensorPublisher.hpp>
#include <Publishers/VectorSensorPublisher.hpp>
#include <Sensor/ISensor.h>
#include <Sensor/IMU/ADXL345.hpp>
#include <Sensor/IMU/L3G4200D.hpp>
#include <Sensor/IMU/HMC5883L.hpp>
#include <Sensor/Range/GP2Y0A21YK.hpp>
#include <Sensor/Range/GP2Y0A710K.hpp>
#include <Sensor/Range/HCSR04.hpp>
#include <Computation/IFilter.h>

int main()
{
    IO::I2CFactory i2CFactory (std::unique_ptr<IO::I2CCreatorBase>(new IO::I2CCreator<IO::WiringPiI2C>));
    auto spi = std::make_shared<IO::WiringPiSPI>(64, 0);
    Computation::FilterFactory filterFactory;
    auto maxBotNode = std::make_shared<MaxBotMessages::MessageBroker>("AHRS", 1);
    auto accelerometer = std::make_shared<Sensor::ADXL345>(i2CFactory);
    auto gyroscope = std::make_shared<Sensor::L3G4200D>(i2CFactory);
    auto magnetometer = std::make_shared<Sensor::HMC5883L>(i2CFactory);
    auto ahrs = std::make_shared<AHRS::AHRS>(maxBotNode);
    auto frontLeftSonic = std::make_shared<Sensor::HCSR04>(5, 6, filterFactory);
    auto frontRightSonic = std::make_shared<Sensor::HCSR04>(1, 4, filterFactory);
    auto frontLeftLeftSonic = std::make_shared<Sensor::HCSR04>(24, 25, filterFactory);
    auto frontRightRightSonic = std::make_shared<Sensor::HCSR04>(22, 23, filterFactory);
    auto rearLeftSonic = std::make_shared<Sensor::HCSR04>(28, 29, filterFactory);
    auto rearRightSonic = std::make_shared<Sensor::HCSR04>(26, 27, filterFactory);
    auto frontDownIR = std::make_shared<Sensor::GP2Y0A21YK>(spi, 0, filterFactory);
    auto rearIR = std::make_shared<Sensor::GP2Y0A21YK>(spi, 2, filterFactory);
    auto frontIR = std::make_shared<Sensor::GP2Y0A710K>(spi, 1, filterFactory);
    auto velocityControl = std::make_shared<Motion::VelocityControl>();

    //accelerometer->Calibrate();
    //gyroscope->Calibrate();
    //magnetometer->Calibrate();
    //ahrs->Calibrate(accelerometer, magnetometer);

    Publishers::VectorSensorPublisher gyroscopePublisher (maxBotNode, "GYRO", "IMU1", gyroscope);
    Publishers::VectorSensorPublisher accelerometerPublisher (maxBotNode, "ACEL", "IMU1", accelerometer);
    Publishers::VectorSensorPublisher magnetometerPublisher (maxBotNode, "MAGN", "IMU1", magnetometer);
    Publishers::RangeSensorPublisher frontRightSonicPublisher (maxBotNode, "RANG", "FRS1", frontRightSonic);
    Publishers::RangeSensorPublisher frontLeftSonicPublisher (maxBotNode, "RANG", "FLS1", frontLeftSonic);
    Publishers::RangeSensorPublisher frontRightRightSonicPublisher (maxBotNode, "RANG", "FRS2", frontRightRightSonic);
    Publishers::RangeSensorPublisher frontLeftLeftSonicPublisher (maxBotNode, "RANG", "FLS2", frontLeftLeftSonic);
    Publishers::RangeSensorPublisher rearLeftSonicPublisher (maxBotNode, "RANG", "RLS1", rearLeftSonic);
    Publishers::RangeSensorPublisher rearRightSonicPublisher (maxBotNode, "RANG", "RRS1", rearRightSonic);
    Publishers::RangeSensorPublisher frontDownIRPublisher (maxBotNode, "RANG", "FIR1", frontDownIR);
    Publishers::RangeSensorPublisher rearIRPublisher (maxBotNode, "RANG", "RIR1", rearIR);
    Publishers::RangeSensorPublisher frontIRPublisher (maxBotNode, "RANG", "FIR2", frontIR);

    double v = 0.1;
    double x=0, y=0, heading=0;
    while(true)
    {
        //std::this_thread::sleep_for(std::chrono::milliseconds(60));
//        gyroscopePublisher.Publish();
//        accelerometerPublisher.Publish();
//        magnetometerPublisher.Publish();
        maxBotNode->DoWork();
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
        velocityControl->SetVelocity(v+=.001, 0);
        velocityControl->RunMotors(x, y, heading);
        std::cout << std::setw(10) << x << std::setw(10) << y << std::setw(10) << heading << std::endl;
    }
}

