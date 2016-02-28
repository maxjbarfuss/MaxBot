#include <IO/WiringPi/WiringPiI2C.hpp>
#include <Packages/I2CPackage.h>
#include <Publishers/VectorSensorPublisher.h>
#include <Sensor/IMU/ADXL345.h>
#include <Sensor/IMU/L3G4200D.h>
#include <Sensor/IMU/HMC5883L.h>

namespace Packages {

///Calibration
//Magnetometer performed 2/13/2016 with all motors running
#define MAG_HARD_IRON_X         171.0           //gauss
#define MAG_HARD_IRON_Y         -253.5          //gauss
#define MAG_HARD_IRON_Z         29.0            //gauss
#define MAG_SOFT_IRON_X         1.0             //scale
#define MAG_SOFT_IRON_Y         0.952381	    //scale
#define MAG_SOFT_IRON_Z         0.95935         //scale

I2CPackage::I2CPackage() : MaxBotPackageBase() {
    IO::I2CFactory i2CFactory (std::unique_ptr<IO::I2CCreatorBase>(new IO::I2CCreator<IO::WiringPiI2C>));
    auto accelerometer = std::make_unique<Sensor::ADXL345>(i2CFactory, Sensor::ADXL345_DATARATE_100_HZ, Sensor::ADXL345_RANGE_8_G);
    auto gyroscope = std::make_unique<Sensor::L3G4200D>(i2CFactory, L3G4200D_REG_SCALE2000);
    auto magnetometer = std::make_unique<Sensor::HMC5883L>(i2CFactory, HMC5883L_SCALE_1_3, HMC5883L_MODE_CONTINUOUS, HMC5883L_RATE_15, Eigen::Vector3d(MAG_HARD_IRON_X, MAG_HARD_IRON_Y, MAG_HARD_IRON_Z), Eigen::Vector3d(MAG_SOFT_IRON_X, MAG_SOFT_IRON_Y, MAG_SOFT_IRON_Z));
    accelerometer->StartCalibration();
    gyroscope->StartCalibration();
    for (int i = 1; i < 128; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (i % 2 == 0)
            accelerometer->StepCalibration(i);
        gyroscope->StepCalibration(i);
    }
    accelerometer->EndCalibration();
    gyroscope->EndCalibration();
    _publishers.push_back(std::make_tuple(std::make_unique<Publishers::VectorSensorPublisher>(_messageNode, "ACEL", "IMU1", std::move(accelerometer)), std::chrono::microseconds(20000), std::chrono::steady_clock::now()));
    _publishers.push_back(std::make_tuple(std::make_unique<Publishers::VectorSensorPublisher>(_messageNode, "GYRO", "IMU1", std::move(gyroscope)), std::chrono::microseconds(10000), std::chrono::steady_clock::now()));
    _publishers.push_back(std::make_tuple(std::make_unique<Publishers::VectorSensorPublisher>(_messageNode, "MAGN", "IMU1", std::move(magnetometer)), std::chrono::microseconds(66667), std::chrono::steady_clock::now()));
}

};
