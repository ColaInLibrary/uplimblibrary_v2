#include "JointPositionActuator.h"

namespace ul {
namespace hal {
JointPositionActuator::JointPositionActuator(const ::std::size_t& dof) : AxisController(dof) {}

JointPositionActuator::~JointPositionActuator() {}
}  // namespace hal
}  // namespace ul
