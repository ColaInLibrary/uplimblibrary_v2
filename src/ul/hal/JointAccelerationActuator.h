#ifndef UL_HAL_JOINTACCELERATIONACTUATOR_H
#define UL_HAL_JOINTACCELERATIONACTUATOR_H

#include <ul/math/Vector.h>

#include "AxisController.h"

namespace ul {
namespace hal {
class JointAccelerationActuator : public virtual AxisController {
 public:
  JointAccelerationActuator(const ::std::size_t& dof);

  virtual ~JointAccelerationActuator();

  virtual void setJointAcceleration(const ::ul::math::Vector& qdd) = 0;

  virtual void setLastJointAcceleration(const ::ul::math::Vector& qdd) = 0;

 protected:
 private:
};
}  // namespace hal
}  // namespace ul

#endif  // UL_HAL_JOINTACCELERATIONACTUATOR_H
