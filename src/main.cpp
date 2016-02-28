#include <iostream>
#include <thread>
#include <chrono>

#include <Packages/IMaxBotPackage.h>
#include <Packages/GPIOPackage.h>
#include <Packages/I2CPackage.h>
#include <Packages/LocalizationPackage.h>
#include <Packages/MotionPackage.h>
#include <Packages/SPIPackage.h>

#include <Dense>
#include <Localization/AHRS.h>

int main()
{
    Packages::I2CPackage i2cPackage;
    Packages::SPIPackage spiPackage;
    Packages::GPIOPackage gpioPackage;
    Packages::MotionPackage motionPackage;
    Packages::LocalizationPackage localizationPackage;
    i2cPackage.Start();
    spiPackage.Start();
    gpioPackage.Start();
    localizationPackage.Start();
    motionPackage.Start();
    std::cin.get();
    i2cPackage.Stop();
    spiPackage.Stop();
    gpioPackage.Stop();
    localizationPackage.Stop();
    motionPackage.Stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

