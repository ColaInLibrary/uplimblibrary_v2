#ifndef UL_HAL_JOINTTORQUEACTUATOR_H
#define UL_HAL_JOINTTORQUEACTUATOR_H

#include <ul/math/Vector.h>

#include "AxisController.h"

namespace ul {
namespace hal {
class JointTorqueActuator : public virtual AxisController {
 public:
  JointTorqueActuator(const ::std::size_t& dof);

  virtual ~JointTorqueActuator();

  virtual void setJointTorque(const ::ul::math::Vector& tau) = 0;

 protected:
 private:
};
}  // namespace hal
}  // namespace ul

#endif  // UL_HAL_JOINTTORQUEACTUATOR_H
