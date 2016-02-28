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

LocalizationPackage::LocalizationPackage() : MaxBotPackageBase() {
    Eigen::Quaterniond imuOrientation;
    imuOrientation.w() = 0;
    imuOrientation.vec() << IMU_ORIENTATION_X, IMU_ORIENTATION_Y, IMU_ORIENTATION_Z;
    Eigen::Quaterniond rot;
    rot = Eigen::AngleAxisd(IMU_ORIENTATION_ROT,  Eigen::Vector3d(IMU_ORIENTATION_X, IMU_ORIENTATION_Y, IMU_ORIENTATION_Z));
    imuOrientation = rot * imuOrientation * rot.inverse();
    _ahrs = std::make_unique<Localization::AHRS>(_messageNode, imuOrientation, Eigen::Vector3d(IMU_MOUNT_X, IMU_MOUNT_Y , IMU_MOUNT_Z), "MAGN", "GYRO", "ACCEL", "WHEL", "CVEL", "AHRS");
}

};
