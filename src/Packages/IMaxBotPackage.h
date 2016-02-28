#pragma once

namespace Packages {

class IMaxBotPackage {
public:
    virtual void Start() = 0;
    virtual void Stop() = 0;
};

};
