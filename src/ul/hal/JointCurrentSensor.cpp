#include "JointCurrentSensor.h"

namespace ul {
namespace hal {
JointCurrentSensor::JointCurrentSensor(const ::std::size_t& dof) : AxisController(dof) {}

JointCurrentSensor::~JointCurrentSensor() {}
}  // namespace hal
}  // namespace ul
