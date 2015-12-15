#pragma once

#include <memory>

#include "IO/II2C.h"

namespace IO {

class I2CCreatorBase {
public:
    virtual ~I2CCreatorBase() {}
    virtual std::unique_ptr<IO::II2C> GetI2C(int deviceAddress) = 0;
};

template<typename T>
class I2CCreator : public I2CCreatorBase
{
protected:
    std::unique_ptr<T> GetI2CTyped(int deviceAddress) {
        return std::unique_ptr<T>(new T(deviceAddress));
    }
public:
    virtual std::unique_ptr<II2C> GetI2C(int deviceAddress){
        return std::move(GetI2CTyped(deviceAddress));
    };
};

class I2CFactory
{
private:
    std::unique_ptr<I2CCreatorBase> _creator;
public:
    I2CFactory(std::unique_ptr<I2CCreatorBase> creator) : _creator(move(creator)) {}

    std::unique_ptr<II2C> GetI2C(int deviceAddress) {
        return _creator->GetI2C(deviceAddress);
    }
};

};
