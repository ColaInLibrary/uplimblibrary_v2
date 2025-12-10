#ifndef UL_HAL_JOINTTORQUESENSOR_H
#define UL_HAL_JOINTTORQUESENSOR_H

#include <ul/math/Vector.h>

#include "AxisController.h"

namespace ul {
namespace hal {
class JointTorqueSensor : public virtual AxisController {
 public:
  JointTorqueSensor(const ::std::size_t& dof);

  virtual ~JointTorqueSensor();

  virtual ::ul::math::Vector getJointTorque() const = 0;

 protected:
 private:
};
}  // namespace hal
}  // namespace ul

#endif  // UL_HAL_JOINTTORQUESENSOR_H
