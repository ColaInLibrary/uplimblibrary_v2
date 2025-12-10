#include "JointVelocitySensor.h"

namespace ul {
namespace hal {
JointVelocitySensor::JointVelocitySensor(const ::std::size_t& dof) : AxisController(dof) {}

JointVelocitySensor::~JointVelocitySensor() {}
}  // namespace hal
}  // namespace ul
