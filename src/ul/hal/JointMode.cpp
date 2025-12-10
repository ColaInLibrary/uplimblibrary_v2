#include "JointMode.h"

namespace ul {
namespace hal {
JointMode::JointMode(const ::std::size_t& dof) : AxisController(dof) {}

JointMode::~JointMode() {}
}  // namespace hal
}  // namespace ul
