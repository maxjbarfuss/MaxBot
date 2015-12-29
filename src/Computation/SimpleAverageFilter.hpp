#pragma once

#include <vector>

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
    SimpleAverageFilter(double min, double max, double maxDeviation, unsigned size)
    : _min(min), _max(max), _maxDeviation(maxDeviation), _size(size), _avg(max + 1) {
        _filter.resize(size);
    }

    virtual void AddValue(double raw) {
        if (_avg > _max) _avg = raw;
        unsigned s = _filter.size();
        if (raw >= _max || raw <= _min) return;
        _avg = (_avg * s + raw) / (s + 1);
        _filter.push_back(raw);
        assert(_filter.size() <= _size);
    }

    virtual void Clear() {
        _filter.clear();
        _avg = _max + 1;
    }

    virtual double GetFilteredValue() {
        double total = 0.0;
        int count = 0;
        for (auto v : _filter ) {
            if (v >= _avg - _maxDeviation && v <= _avg + _maxDeviation) {
                total += v;
                count ++;
            }
        }
        return total / count;
    }
};

};
