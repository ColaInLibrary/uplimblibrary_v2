#include "JointTorqueSensor.h"

namespace ul {
namespace hal {
JointTorqueSensor::JointTorqueSensor(const ::std::size_t& dof) : AxisController(dof) {}

JointTorqueSensor::~JointTorqueSensor() {}
}  // namespace hal
}  // namespace ul
