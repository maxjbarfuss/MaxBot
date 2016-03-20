#pragma once

#include <vector>
#include <cassert>

#include <Computation/IFilter.h>

namespace Computation {

template<class T>
class SimpleAverageFilter : public IFilter<T> {
private:
    T _min;
    T _max;
    T _maxDeviation;
    unsigned _size;
    std::vector<T> _filter;
    T _avg;
    T _defaultValue;
public:
    SimpleAverageFilter(T min, T max, T maxDeviation, unsigned size, T defaultValue)
    : _min(min), _max(max), _maxDeviation(maxDeviation), _size(size), _avg(max + 1), _defaultValue(defaultValue) {
        _filter.resize(size);
    }

    virtual void AddValue(T raw) {
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

    virtual T GetFilteredValue() {
        T total = _defaultValue;
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
