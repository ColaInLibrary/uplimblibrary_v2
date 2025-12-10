#include "JointAccelerationActuator.h"

namespace ul {
namespace hal {
JointAccelerationActuator::JointAccelerationActuator(const ::std::size_t& dof) : AxisController(dof) {}

JointAccelerationActuator::~JointAccelerationActuator() {}
}  // namespace hal
}  // namespace ul
