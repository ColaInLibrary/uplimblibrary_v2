#ifndef UL_HAL_JOINTVELOCITYACTUATOR_H
#define UL_HAL_JOINTVELOCITYACTUATOR_H

#include <ul/math/Vector.h>

#include "AxisController.h"

namespace ul {
namespace hal {
class JointVelocityActuator : public virtual AxisController {
 public:
  JointVelocityActuator(const ::std::size_t& dof);

  virtual ~JointVelocityActuator();

  virtual void setJointVelocity(const ::ul::math::Vector& qd) = 0;

  virtual void setLastJointVelocity(const ::ul::math::Vector& qd) = 0;

 protected:
 private:
};
}  // namespace hal
}  // namespace ul

#endif  // UL_HAL_JOINTVELOCITYACTUATOR_H
