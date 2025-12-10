#ifndef UL_HAL_JOINTCURRENTSENSOR_H
#define UL_HAL_JOINTCURRENTSENSOR_H

#include <ul/math/Vector.h>

#include "AxisController.h"

namespace ul {
namespace hal {
class JointCurrentSensor : public virtual AxisController {
 public:
  JointCurrentSensor(const ::std::size_t& dof);

  virtual ~JointCurrentSensor();

  virtual ::ul::math::Vector getJointCurrent() const = 0;

 protected:
 private:
};
}  // namespace hal
}  // namespace ul

#endif  // UL_HAL_JOINTCURRENTSENSOR_H
