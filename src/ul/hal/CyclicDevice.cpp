#include "CyclicDevice.h"

namespace ul {
namespace hal {
CyclicDevice::CyclicDevice(const ::ul::math::Real& updateRate) : updateRate(updateRate) {}

CyclicDevice::~CyclicDevice() {}

::ul::math::Real CyclicDevice::getUpdateRate() const { return this->updateRate; }

void CyclicDevice::setUpdateRate(const ::ul::math::Real & updateRate) { this->updateRate = updateRate; }
}  // namespace hal
}  // namespace ul
