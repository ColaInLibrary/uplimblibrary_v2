#include "JointPositionSensor.h"

namespace ul {
namespace hal {
JointPositionSensor::JointPositionSensor(const ::std::size_t& dof) : AxisController(dof) {}

JointPositionSensor::~JointPositionSensor() {}
}  // namespace hal
}  // namespace ul
