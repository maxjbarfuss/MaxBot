#include <Computation/SimpleAverageFilter.h>

namespace Computation {

SimpleAverageFilter::SimpleAverageFilter(double min, double max, double maxDeviation, unsigned size)
: _min(min), _max(max), _maxDeviation(maxDeviation), _size(size), _avg(max + 1) {
    _filter.resize(size);
}

void SimpleAverageFilter::AddValue(double raw) {
    if (_avg > _max) _avg = raw;
    unsigned s = _filter.size();
    if (raw >= _max || raw <= _min) return;
    _avg = (_avg * s + raw) / (s + 1);
    _filter.push_back(raw);
    assert(_filter.size() <= _size);
}

void SimpleAverageFilter::Clear() {
    _filter.clear();
    _avg = _max + 1;
}

double SimpleAverageFilter::GetFilteredValue() {
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
