#ifndef UL_HAL_JOINTMODEOPERATION_H
#define UL_HAL_JOINTMODEOPERATION_H

#include <ul/math/Vector.h>

#include "AxisController.h"
#include <vector>

namespace ul {
namespace hal {
class JointModeOperation : public virtual AxisController {
 public:
  JointModeOperation(const ::std::size_t& dof);

  virtual ~JointModeOperation();

  virtual void setJointModeOperation(const ::std::vector<uint8_t>& mode_operation) = 0;

 protected:
 private:
};
}  // namespace hal
}  // namespace ul

#endif  // UL_HAL_JOINTMODEOPERATION_H
