#ifndef UL_HAL_JOINTMODE_H
#define UL_HAL_JOINTMODE_H

#include <ul/math/Vector.h>

#include "AxisController.h"
#include <vector>

namespace ul {
namespace hal {
class JointMode : public virtual AxisController {
 public:
  JointMode(const ::std::size_t& dof);

  virtual ~JointMode();

  virtual ::std::vector<::std::uint8_t> getJointMode() const = 0;

 protected:
 private:
};
}  // namespace hal
}  // namespace ul

#endif  // UL_HAL_JOINTMODE_H
