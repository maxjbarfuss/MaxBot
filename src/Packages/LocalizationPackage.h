#pragma once

#include <Localization/AHRS.h>
#include <Packages/MaxBotPackageBase.h>

namespace Packages {

class LocalizationPackage : public MaxBotPackageBase {
private:
    std::unique_ptr<Localization::AHRS> _ahrs;
public:
    LocalizationPackage();
};

};
