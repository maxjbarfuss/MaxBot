#include <Packages/GPIOPackage.h>
#include <Publishers/RangeSensorPublisher.h>
#include <Sensor/Range/HCSR04.h>

namespace Packages {

GPIOPackage::GPIOPackage() : MaxBotPackageBase() {
    _publishers.push_back(std::make_tuple(std::make_unique<Publishers::RangeSensorPublisher>(_messageNode, "RANG", "FRS1", std::make_unique<Sensor::HCSR04>(5, 6)), std::chrono::microseconds(60000), std::chrono::steady_clock::now()));
    _publishers.push_back(std::make_tuple(std::make_unique<Publishers::RangeSensorPublisher>(_messageNode, "RANG", "FLS1", std::make_unique<Sensor::HCSR04>(1, 4)), std::chrono::microseconds(60000), std::chrono::steady_clock::now()));
    _publishers.push_back(std::make_tuple(std::make_unique<Publishers::RangeSensorPublisher>(_messageNode, "RANG", "FRS2", std::make_unique<Sensor::HCSR04>(24, 25)), std::chrono::microseconds(60000), std::chrono::steady_clock::now()));
    _publishers.push_back(std::make_tuple(std::make_unique<Publishers::RangeSensorPublisher>(_messageNode, "RANG", "FLS2", std::make_unique<Sensor::HCSR04>(22, 23)), std::chrono::microseconds(60000), std::chrono::steady_clock::now()));
    _publishers.push_back(std::make_tuple(std::make_unique<Publishers::RangeSensorPublisher>(_messageNode, "RANG", "RLS1", std::make_unique<Sensor::HCSR04>(28, 29)), std::chrono::microseconds(60000), std::chrono::steady_clock::now()));
    _publishers.push_back(std::make_tuple(std::make_unique<Publishers::RangeSensorPublisher>(_messageNode, "RANG", "RRS1", std::make_unique<Sensor::HCSR04>(26, 27)), std::chrono::microseconds(60000), std::chrono::steady_clock::now()));
}

};
