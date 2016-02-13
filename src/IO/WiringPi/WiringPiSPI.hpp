#pragma once

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <mcp3004.h>

#include "IO/ISPI.h"

namespace IO {

class WiringPiSPI : public ISPI {
private:
    uint16_t _base;

public:
    WiringPiSPI(uint16_t base, uint16_t channel) : _base(base) {
        mcp3004Setup (_base, channel);
    }

    virtual uint16_t Read(uint8_t pin) {
        return analogRead(_base + pin);
    }
};

};
