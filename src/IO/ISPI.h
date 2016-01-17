#pragma once

#include <stdint.h>

namespace IO {

class ISPI {
public:
    virtual uint16_t Read(uint8_t pin) = 0;
};

};
