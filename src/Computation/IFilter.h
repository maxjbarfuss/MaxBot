#pragma once

namespace Computation {

class IFilter {
public:
    virtual double GetFilteredValue(double raw) = 0;
};

};
