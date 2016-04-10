#include <Dense>
#include <Packages/LocalizationPackage.h>

namespace Packages {

//Origin @ point equidistant from all 4 wheels on the ground plane
#define IMU_MOUNT_X             -.015           //meters
#define IMU_MOUNT_Y             0.0             //meters
#define IMU_MOUNT_Z             .19125          //meters
//right hand coordinates unit, rotate around a vector from the origin
#define IMU_ORIENTATION_ROT     M_PI
#define IMU_ORIENTATION_X       1.0
#define IMU_ORIENTATION_Y       0.0
#define IMU_ORIENTATION_Z       0.0
#define GYRO_ACCURACY           .95
#define MAGNETOMETER_ACCURACY   .02

LocalizationPackage::LocalizationPackage() : MaxBotPackageBase() {
    Eigen::Quaterniond imuOrientation;
    imuOrientation.w() = 0;
    imuOrientation.vec() << IMU_ORIENTATION_X, IMU_ORIENTATION_Y, IMU_ORIENTATION_Z;
    Eigen::Quaterniond rot;
    rot = Eigen::AngleAxisd(IMU_ORIENTATION_ROT,  Eigen::Vector3d(IMU_ORIENTATION_X, IMU_ORIENTATION_Y, IMU_ORIENTATION_Z));
    imuOrientation = rot * imuOrientation * rot.inverse();
    _ahrs = std::make_shared<Localization::AHRS>(_messageNode, "AHRS");
    _publishers.push_back(std::make_tuple(_ahrs, std::chrono::microseconds(5000), std::chrono::steady_clock::now()));
    _gyro = std::make_unique<Subscribers::AngularRate3dSubscriber>(_messageNode, "GYRO", _ahrs, GYRO_ACCURACY);
    _magnetometer = std::make_unique<Subscribers::MagnetometerSubscriber>(_messageNode, "MAGN", "ACEL", _ahrs, MAGNETOMETER_ACCURACY);
}

};
