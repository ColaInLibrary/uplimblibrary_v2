#ifndef UL_HAL_JOINTPOSITIONACTUATOR_H
#define UL_HAL_JOINTPOSITIONACTUATOR_H

#include <ul/math/Vector.h>

#include "AxisController.h"

namespace ul {
namespace hal {
class JointPositionActuator : public virtual AxisController {
 public:
  JointPositionActuator(const ::std::size_t& dof);

  virtual ~JointPositionActuator();

  virtual void setJointPosition(const ::ul::math::Vector& q) = 0;

  virtual void setLastJointPosition(const ::ul::math::Vector& q) = 0;

 protected:
 private:
};
}  // namespace hal
}  // namespace ul

#endif  // UL_HAL_JOINTPOSITIONACTUATOR_H
