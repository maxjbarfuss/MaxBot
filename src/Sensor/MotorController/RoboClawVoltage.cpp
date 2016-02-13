#include <Sensor/MotorController/RoboClawVoltage.h>

namespace Sensor {

    RoboClawVoltage::RoboClawVoltage(std::shared_ptr<MotionControl::IRoboClaw> motorController, int address) : _motorController(motorController), _address(address) {}

    double RoboClawVoltage::GetReading() {
        bool valid = false;
        auto voltage = _motorController->ReadMainBatteryVoltage(_address, &valid);
        if (valid)
            return voltage * ROBO_CLAW_VOLTAGE_SCALE;
        return NAN;
    }

};
