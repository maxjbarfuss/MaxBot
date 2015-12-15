#pragma once

#include <stdint.h>

namespace IO {

class II2C {
    public:
        virtual ~II2C() {}
        virtual void Write8(uint8_t registerAddress, uint8_t data) = 0;
        virtual int8_t Read8(uint8_t registerAddress) = 0;
        virtual int16_t Read10(uint8_t registerAddress) = 0;
        virtual int16_t Read16(uint8_t registerAddress) = 0;
        virtual int16_t Read16Signed(uint8_t registerAddress) = 0;
};

};

