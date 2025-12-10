#include "JointVelocityActuator.h"

namespace ul {
namespace hal {
JointVelocityActuator::JointVelocityActuator(const ::std::size_t& dof) : AxisController(dof) {}

JointVelocityActuator::~JointVelocityActuator() {}
}  // namespace hal
}  // namespace ul
