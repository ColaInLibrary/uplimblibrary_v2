/**
 ******************************************************************************
 * @Description   :
 * @author        : Yuan Hanqing
 * @Date          : 25-5-14
 * @Version       : 0.0.1
 * @File          : Teach.cpp
 ******************************************************************************
 */

#include "Teach.h"

namespace ul {
namespace controller {
Teach::Teach(::ul::hal::Coach& d, ::ul::mdl::Dynamic& r)
    : driver(d),
      robot(r),
      dof(r.getDof()) {}

Teach::~Teach() {}

bool Teach::teachMode(::ul::std17::RobotCommand& cmd) {
  if (::ul::std17::RobotCommand::NO_CMD == cmd) {
    // std::vector<uint8_t> mode_operation(dof, 10);
    // ::ul::math::Vector cmd_tau(dof);
    // cmd_tau.setZero();
    // this->driver.setJointModeOperation(mode_operation);
    // this->driver.setJointTorque(cmd_tau);
    std::cout << YELLOW << "Enters the teach mode!" << RESET << std::endl;
    return true;
  } else {
    return false;
  }
}

bool Teach::endTeachMode(::ul::std17::RobotCommand& cmd) {
  if (::ul::std17::RobotCommand::TEACH == cmd) {
    // std::vector<uint8_t> mode_operation(dof, 8);
    // this->driver.setJointModeOperation(mode_operation);
    // this->driver.setJointPosition(this->driver.getJointPosition());
    std::cout << YELLOW << "Out the teach mode!" << RESET << std::endl;
    return true;
  } else {
    return false;
  }
}

bool Teach::teachTorqueGenerate() {
  ::ul::math::Vector cmd_tau(dof), act_q(dof), cmd_q_last(dof);
  this->robot.setPosition(this->driver.getJointPosition());
  this->robot.calculateGravity();
  cmd_tau = this->robot.getGravity();
  act_q = this->driver.getJointPosition();
  cmd_q_last = this->driver.getLastCommandJointPosition();

  std::vector<uint8_t> cmd_mode_operation = this->driver.getCommandJointModeOperation();
  for(size_t i = 0; i < cmd_mode_operation.size(); ++i) {
    if(cmd_mode_operation[i] != 10) {
      cmd_tau[i] = 0;
      act_q[i] = cmd_q_last[i];
    }
  }

  this->driver.setJointTorque(cmd_tau);
  this->driver.setJointPosition(act_q);

  ::ul::math::Vector temp(dof);
  temp.setZero();
  this->driver.setJointVelocity(temp);
  this->driver.setJointAcceleration(temp);
  return true;
}

}  // namespace controller
}  // namespace ul