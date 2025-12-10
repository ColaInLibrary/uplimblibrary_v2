#include "JointTorqueActuator.h"

namespace ul {
namespace hal {
JointTorqueActuator::JointTorqueActuator(const ::std::size_t& dof) : AxisController(dof) {}

JointTorqueActuator::~JointTorqueActuator() {}
}  // namespace hal
}  // namespace ul
