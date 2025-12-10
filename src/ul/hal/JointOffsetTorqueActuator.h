#ifndef UL_HAL_JOINTOFFSETTORQUEACTUATOR_H
#define UL_HAL_JOINTOFFSETTORQUEACTUATOR_H

#include <ul/math/Vector.h>

#include "AxisController.h"

namespace ul {
namespace hal {
class JointOffsetTorqueActuator : public virtual AxisController {
 public:
  JointOffsetTorqueActuator(const ::std::size_t& dof);

  virtual ~JointOffsetTorqueActuator();

  virtual void setJointOffsetTorqueActuator(const ::ul::math::Vector& offset_tau) = 0;

 protected:
 private:
};
}  // namespace hal
}  // namespace ul

#endif  // UL_HAL_JOINTOFFSETTORQUEACTUATOR_H
