#pragma once

#include <vector>
#include <cassert>

#include <Computation/IFilter.h>

namespace Computation {

class SimpleAverageFilter : public IFilter {
private:
    double                      _min;
    double                      _max;
    double                      _maxDeviation;
    unsigned                    _size;
    std::vector<double>         _filter;
    double                      _avg;
public:
    SimpleAverageFilter(double min, double max, double maxDeviation, unsigned size);
    virtual void AddValue(double raw);
    virtual void Clear();
    virtual double GetFilteredValue();
};

};
