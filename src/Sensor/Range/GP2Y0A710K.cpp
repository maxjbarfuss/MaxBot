#include <cmath>
#include <thread>
#include <chrono>

#include <Computation/FilterFactory.hpp>
#include <Sensor/Range/GP2Y0A710K.h>

namespace Sensor {

GP2Y0A710K::GP2Y0A710K(std::shared_ptr<IO::ISPI> spi, uint8_t pin) : _spi(spi), _pin(pin) {
    _filter = std::move(Computation::FilterFactory<double>::GetSimpleFilter(GP2Y0A710K_MIN_RANGE, GP2Y0A710K_MAX_RANGE, GP2Y0A710K_MAX_DEVIATION, GP2Y0A710K_FILTER_SIZE, 0));
}

double GP2Y0A710K::GetReading() {
    _filter->Clear();
    for (int i=0; i<GP2Y0A710K_FILTER_SIZE; i++) {
        _filter->AddValue(GP2Y0A710K_FIT_ALPHA * pow(_spi->Read(_pin), GP2Y0A710K_FIT_BETA));
        std::this_thread::sleep_for(std::chrono::microseconds(GP2Y0A710K_READ_DELAY));
    }
    return _filter->GetFilteredValue();
}

};
