#pragma once

#include <memory>

#include <Computation/IFilter.h>
#include <Computation/SimpleAverageFilter.hpp>

namespace Computation {

template <class T>
class FilterFactory {
public:
    static std::unique_ptr<IFilter<T>> GetSimpleFilter(T min, T max, T max_deviation, unsigned size, T defaultValue) {
        return std::make_unique<SimpleAverageFilter<double>>(min, max, max_deviation, size, defaultValue);
    }
};

};
