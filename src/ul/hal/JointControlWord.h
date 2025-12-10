#ifndef UL_HAL_JOINTCONTROLWORD_H
#define UL_HAL_JOINTCONTROLWORD_H

#include <ul/math/Vector.h>
#include "AxisController.h"
#include <vector>

namespace ul {
namespace hal {
class JointControlWord : public virtual AxisController {
 public:
  JointControlWord(const ::std::size_t& dof);

  virtual ~JointControlWord();

  virtual void setJointControlWord(const ::std::vector<uint16_t>& ctrlwd) = 0;

 protected:
 private:
};
}  // namespace hal
}  // namespace ul

#endif  // UL_HAL_JOINTCONTROLWORD_H
