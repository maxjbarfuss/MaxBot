#pragma once

namespace Computation {

class IFilter {
public:
    virtual void AddValue(double raw) = 0;
    virtual void Clear() = 0;
    virtual double GetFilteredValue() = 0;
};

};
