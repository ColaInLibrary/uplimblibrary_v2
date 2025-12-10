#ifndef UL_HAL_JOINTVELOCITYSENSOR_H
#define UL_HAL_JOINTVELOCITYSENSOR_H

#include <ul/math/Vector.h>

#include "AxisController.h"

namespace ul {
namespace hal {
class JointVelocitySensor : public virtual AxisController {
 public:
  JointVelocitySensor(const ::std::size_t& dof);

  virtual ~JointVelocitySensor();

  virtual ::ul::math::Vector getJointVelocity() const = 0;

 protected:
 private:
};
}  // namespace hal
}  // namespace ul

#endif  // UL_HAL_JOINTVELOCITYSENSOR_H
