#ifndef UL_HAL_JOINTSTATUSWORD_H
#define UL_HAL_JOINTSTATUSWORD_H

#include <ul/math/Vector.h>

#include "AxisController.h"
#include <vector>

namespace ul {
namespace hal {
class JointStatusWord : public virtual AxisController {
 public:
  JointStatusWord(const ::std::size_t& dof);

  virtual ~JointStatusWord();

  virtual void getJointStatusWord(const ::std::vector<::std::uint16_t>& statuswd) = 0;

 protected:
 private:
};
}  // namespace hal
}  // namespace ul

#endif  // UL_HAL_JOINTSTATUSWORD_H
