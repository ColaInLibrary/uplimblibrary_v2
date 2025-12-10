/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-11-12
 * @Version       : 0.0.1
 * @File          : Controller.h
 ******************************************************************************
 */

#ifndef UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_CONTROLLER_H_
#define UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_CONTROLLER_H_

#include <mujoco/mujoco.h>

class MjController {
public:
  enum ControlMode {
    TORQUE_CONTROL,
    POSITION_CONTROL
  };

  MjController();
  void setControlMode(ControlMode mode) { controlMode_ = mode; }
  void torqueControl(const mjModel* m, mjData* d);
  void positionControl(const mjModel* m, mjData* d);

private:
  ControlMode controlMode_ = TORQUE_CONTROL;
};

#endif // UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_CONTROLLER_H_
