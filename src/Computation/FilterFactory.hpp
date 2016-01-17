#pragma once

#include <memory>

#include <Computation/IFilter.h>
#include <Computation/SimpleAverageFilter.h>

namespace Computation {

class FilterFactory {
public:
    std::unique_ptr<IFilter> GetFilter(double min, double max, double max_deviation, int size) {
        return std::make_unique<SimpleAverageFilter>(min, max, max_deviation, size);
    }
};

};
