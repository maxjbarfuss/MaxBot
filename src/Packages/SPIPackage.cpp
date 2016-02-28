#include <Computation/FilterFactory.hpp>
#include <IO/WiringPi/WiringPiSPI.hpp>
#include <Packages/SPIPackage.h>
#include <Publishers/RangeSensorPublisher.h>
#include <Sensor/Range/GP2Y0A02YK.h>
#include <Sensor/Range/GP2Y0A21YK.h>
#include <Sensor/Range/GP2Y0A710K.h>

namespace Packages {

SPIPackage::SPIPackage() : MaxBotPackageBase() {
    auto spi = std::make_shared<IO::WiringPiSPI>(64, 0);
    Computation::FilterFactory filterFactory;
    auto frontDownIR = std::make_unique<Sensor::GP2Y0A21YK>(spi, 0, filterFactory);
    auto rearIR = std::make_unique<Sensor::GP2Y0A21YK>(spi, 2, filterFactory);
    auto frontIR = std::make_unique<Sensor::GP2Y0A710K>(spi, 1, filterFactory);
    auto frontLeftIR = std::make_unique<Sensor::GP2Y0A02YK>(spi,4 , filterFactory);
    auto frontRightIR = std::make_unique<Sensor::GP2Y0A02YK>(spi, 5, filterFactory);
    _publishers.push_back(std::make_tuple(std::make_unique<Publishers::RangeSensorPublisher>(_messageNode, "RANG", "FIR1", std::move(frontDownIR)), std::chrono::microseconds(10000), std::chrono::steady_clock::now()));
    _publishers.push_back(std::make_tuple(std::make_unique<Publishers::RangeSensorPublisher>(_messageNode, "RANG", "RIR1", std::move(rearIR)), std::chrono::microseconds(10000), std::chrono::steady_clock::now()));
    _publishers.push_back(std::make_tuple(std::make_unique<Publishers::RangeSensorPublisher>(_messageNode, "RANG", "FIR2", std::move(frontIR)), std::chrono::microseconds(10000), std::chrono::steady_clock::now()));
    _publishers.push_back(std::make_tuple(std::make_unique<Publishers::RangeSensorPublisher>(_messageNode, "RANG", "FIR3", std::move(frontLeftIR)), std::chrono::microseconds(10000), std::chrono::steady_clock::now()));
    _publishers.push_back(std::make_tuple(std::make_unique<Publishers::RangeSensorPublisher>(_messageNode, "RANG", "FIR4", std::move(frontRightIR)), std::chrono::microseconds(10000), std::chrono::steady_clock::now()));
 }

};
