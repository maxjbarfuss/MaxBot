#pragma once

#include <wiringPiI2C.h>

#include "IO/II2C.h"

namespace IO {

class WiringPiI2C : public II2C {
    private:
        int _deviceId;
    public:
        WiringPiI2C(uint16_t device_address) {
            _deviceId = wiringPiI2CSetup(device_address);
        }

        virtual void Write8(uint8_t register_address, uint8_t data) {
            wiringPiI2CWriteReg8(_deviceId, register_address, data);
        };

        virtual int8_t Read8(uint8_t register_address) {
            return wiringPiI2CReadReg8(_deviceId, register_address);
        };

        virtual int16_t Read10(uint8_t register_address) {
            uint8_t hi = wiringPiI2CReadReg8(_deviceId, register_address);
            uint8_t lo = wiringPiI2CReadReg8(_deviceId, register_address + 1);
            int16_t v = ((lo << 8)) | hi;
            if(v & (1 << (16 - 1)))
                v = v - (1 << 16);
            return v;
        };

        virtual int16_t Read16Signed(uint8_t register_address) {
            uint8_t lo = wiringPiI2CReadReg8(_deviceId, register_address);
            uint8_t hi = wiringPiI2CReadReg8(_deviceId, register_address + 1);
            uint16_t v = (hi << 8) | lo;
            if (v & (1 << 15))
                return v | ~65535;
            else
                return v & 65535;
        };

        virtual int16_t Read16(uint8_t register_address) {
            uint8_t hi = wiringPiI2CReadReg8(_deviceId, register_address);
            uint8_t lo = wiringPiI2CReadReg8(_deviceId, register_address + 1);
            return (hi << 8) + lo;
        };
};

};
