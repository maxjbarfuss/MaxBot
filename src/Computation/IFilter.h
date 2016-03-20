#pragma once

namespace Computation {

template <class T>
class IFilter {
public:
    virtual void AddValue(T raw) = 0;
    virtual void Clear() = 0;
    virtual T GetFilteredValue() = 0;
};

};
