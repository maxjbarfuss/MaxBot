#include <Dense>

#include <Packages/LocalizationPackage.h>

namespace Packages {

//Origin @ point equidistant from all 4 wheels on the ground plane
#define IMU_MOUNT_X             -.015           //meters
#define IMU_MOUNT_Y             0.0             //meters
#define IMU_MOUNT_Z             .19125          //meters
//right hand coordinates unit, rotate around a vector from the origin
#define IMU_ORIENTATION_X               M_PI + 1.5*M_PI/360
#define IMU_ORIENTATION_Y               1.7*M_PI/360
#define IMU_ORIENTATION_Z               0.0
#define GYRO_ACCURACY                   .99
#define COMPLIMENTARY_FILTER_ACCURACY   .02
#define WHEEL_SPEED_ACCURACY            .2

LocalizationPackage::LocalizationPackage() : MaxBotPackageBase() {
    Eigen::Quaterniond rotation;
    rotation = Eigen::Matrix3d(Eigen::AngleAxisd(IMU_ORIENTATION_X, Eigen::Vector3d::UnitX()))
             * Eigen::Matrix3d(Eigen::AngleAxisd(IMU_ORIENTATION_Y, Eigen::Vector3d::UnitY()))
             * Eigen::Matrix3d(Eigen::AngleAxisd(IMU_ORIENTATION_Z, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d translation(IMU_MOUNT_X, IMU_MOUNT_Y, IMU_MOUNT_Z);
    _pose = std::make_shared<Localization::Pose>(_messageNode, "POSE", "SPOSE", "SPOS", "SORI");
    _publishers.push_back(std::make_tuple(_pose, std::chrono::microseconds(50000), std::chrono::steady_clock::now()));
    _gyro = std::make_shared<Subscribers::AngularRate3dSubscriber>(_messageNode, "SORI", "GYRO", rotation, translation, GYRO_ACCURACY);
    _complimentaryFilter = std::make_shared<Subscribers::ComplimentaryFilterSubscriber>(_messageNode, "SORI", "MAGN", "ACEL", rotation, translation, COMPLIMENTARY_FILTER_ACCURACY);
    _wheelSpeed = std::make_shared<Subscribers::WheelSpeedSubscriber>(_messageNode, "SPOSE", "WHEL", "POSE", WHEEL_SPEED_ACCURACY);
}

};
