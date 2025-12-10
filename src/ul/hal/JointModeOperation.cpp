#include "JointModeOperation.h"

namespace ul {
namespace hal {
JointModeOperation::JointModeOperation(const ::std::size_t& dof) : AxisController(dof) {}

JointModeOperation::~JointModeOperation() {}
}  // namespace hal
}  // namespace ul
