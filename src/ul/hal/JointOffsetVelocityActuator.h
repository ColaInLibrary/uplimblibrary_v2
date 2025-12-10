/*
 * @Description:  
 * @Author: AN Hao
 * @Date: 2025-01-16 11:18:55
 * @Version: 0.0.1
 * @LastEditors: AN Hao
 * @LastEditTime: 2025-01-16 17:09:06
 * @FilePath: /UplimbLibrary/src/ul/hal/JointOffsetVelocityActuator.h
 */
#ifndef UL_HAL_JOINTOFFSETVELOCITYACYUATOR_H
#define UL_HAL_JOINTOFFSETVELOCITYACYUATOR_H

#include <ul/math/Vector.h>

#include "AxisController.h"

namespace ul {
namespace hal {
class JointOffsetVelocityActuator : public virtual AxisController {
 public:
  JointOffsetVelocityActuator(const ::std::size_t& dof);

  virtual ~JointOffsetVelocityActuator();

  virtual void setJointOffsetVelocityActuator(const ::ul::math::Vector& offset_vel) = 0;

 protected:
 private:
};
}  // namespace hal
}  // namespace ul

#endif  // UL_HAL_JOINTOFFSETVELOCITYACYUATOR_H
