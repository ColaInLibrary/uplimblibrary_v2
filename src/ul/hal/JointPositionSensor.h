#ifndef UL_HAL_JOINTPOSITIONSENSOR_H
#define UL_HAL_JOINTPOSITIONSENSOR_H

#include <ul/math/Vector.h>

#include "AxisController.h"

namespace ul {
namespace hal {
class JointPositionSensor : public virtual AxisController {
 public:
  JointPositionSensor(const ::std::size_t& dof);

  virtual ~JointPositionSensor();

  virtual ::ul::math::Vector getJointPosition() const = 0;

  virtual ::ul::math::Vector getLastJointPosition() const = 0;

 protected:
 private:
};
}  // namespace hal
}  // namespace ul

#endif  // UL_HAL_JOINTPOSITIONSENSOR_H
