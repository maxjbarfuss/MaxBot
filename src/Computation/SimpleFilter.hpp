#pragma once

#include <cmath>

#include <Computation/IFilter.h>

namespace Computation {

class SimpleFilter : public IFilter {
private:
    double                  _min;
    double                  _max;
    double                  _maxDeviation;
    int                     _size;
    std::vector<double>     _filter;

    double AverageFilter() {
        double total = 0.0;
        int count = 0;
        for (int i = _size - 1; i >= 0; i--) {
            if (_filter[i] <= _max && _filter[i] >= _min) {
                    total += _filter[i];
                    count += i;
            }
        }
        return total / count;
    }

public:
    SimpleFilter(double min, double max, double maxDeviation, int size)
    : _min(min), _max(max), _maxDeviation(maxDeviation), _size(size) {
        _filter.resize(_size);
        for (int i = 0; i<_size; i++)
            _filter[i] = _max + 1;
    }

    virtual double GetFilteredValue(double raw) {
        double total = 0.0;
        int count = 0;
        double avg = AverageFilter();
        for (int i = _size; i >= 0; i--) {
            double v = (i < _size) ? _filter[i] : raw;
            if (v <= _max && v >= _min && v >= avg - _maxDeviation && v <= avg + _maxDeviation) {

                    total += v * i;
                    count += i;
            }
            if (i < _size - 1)
                _filter[i] = _filter[i+1];
        }
        _filter[_size - 1] = raw;
        return total / count;
    }
};

};
