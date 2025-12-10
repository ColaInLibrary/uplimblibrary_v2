/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-2-19
 * @Version       : 0.0.1
 * @File          : JointsMotion.cpp
 ******************************************************************************
 */

#include "JointsSecurity.h"

namespace ul {
namespace controller {
JointsSecurity::JointsSecurity( const ::std::size_t& dof)
    : cmd_q(dof),
      cmd_qd(dof),
      cmd_qdd(dof),
      cmd_qddd(dof),
      cmd_q_last(dof),
      cmd_qd_last(dof),
      dof(dof),
      dt(0.001),
      lower_pos_limit(dof),
      upper_pos_limit(dof),
      joint_pos_limit_flag(dof),
      joint_vel_limit_flag(dof),
      joint_tau_limit_flag(dof) {}

JointsSecurity::~JointsSecurity() {}

void JointsSecurity::setJointPositionLimit(const ::ul::math::Vector& lower_limit, const ::ul::math::Vector& upper_limit) {
  this->lower_pos_limit = lower_limit;
  this->upper_pos_limit = upper_limit;
}

void JointsSecurity::getJointPositionLimit(::ul::math::Vector& lower_limit, ::ul::math::Vector& upper_limit) const {
  lower_limit = this->lower_pos_limit;
  upper_limit = this->upper_pos_limit;
}

void JointsSecurity::setJointPositionThreshold(const double pos_threshold) {
  this->pos_threshold = pos_threshold;
}

void JointsSecurity::getJointPositionThreshold(double& pos_threshold) const {
  pos_threshold = this->pos_threshold;
}

bool JointsSecurity::boundNormalize(const double &lb, const double &ub, const double &epsilon, const double &x0, double &x) {
  assert(lb < ub);

  const double extended_lb = lb - epsilon;
  const double extended_ub = ub + epsilon;

  x = 2/(extended_ub - extended_lb)*(x0 - (extended_lb + extended_ub)/2);
  return true;
}

/**
 * 判断关节位置是否安全
 * @param q ::ul::math
 * @return 到达限位为true，否则为false
 */
bool JointsSecurity::isJointsWithSafetyLimit(const ::ul::math::Vector& q) {
  bool is_limit = false;
  ::ul::math::Vector joints_norm(this->dof);
  for (int j = 0; j < this->dof; ++j) {
    this->joint_pos_limit_flag[j] = false;
    boundNormalize(this->lower_pos_limit[j], this->upper_pos_limit[j], 1e-6, q[j], joints_norm[j]);
    if (abs(joints_norm[j])> 1) {  // 大于1的时候认为是危险区域, joint_pos_limit_flag置1，否则置0
      this->joint_pos_limit_flag[j] = true;
      is_limit = true;
    }
  }
  return is_limit;
}

int JointsSecurity::jointsPosLimitProtection(bool protect) {
  int flag = 0;
  this->cmd_q = this->cmd_q_last;
//  this->act_q = this->driver.getJointPosition();  // TODO: 等待V2完善双缓冲机制
//  this->cmd_q_last = this->driver.getLastCommandJointPosition();
//  this->cmd_qd_last = this->driver.getLastCommandJointVelocity();
  if (isJointsWithSafetyLimit(cmd_q)) {  //位置输出约束
    flag += 1;
    if (protect) {
//      this->cmd_qd = this->driver.getCommandJointVelocity();  // TODO: 待完善双缓冲机制
//      this->cmd_qdd = this->driver.getCommandJointAcceleration();
      for (int j = 0; j < this->dof; ++j) {
        if (this->joint_pos_limit_flag[j]) {
          if (cmd_q[j] > this->upper_pos_limit[j]) {
            // ::std::cout << RED << " Joint[" << j << "] cmd_pos: " << cmd_q[j] << " is out of Algorithm UPPER limit: " << this->upper_pos_limit[j] << RESET << std::endl;
            cmd_q[j] = this->upper_pos_limit[j];
          }
          if (cmd_q[j] < this->lower_pos_limit[j]) {
            // ::std::cout << RED << " Joint[" << j << "] cmd_pos: " << cmd_q[j] << " is out of Algorithm LOWER limit: " << this->lower_pos_limit[j] << RESET << std::endl;
            cmd_q[j] = this->lower_pos_limit[j];
          }
          cmd_qd[j] = 0.0;
          cmd_qdd[j] = 0.0;
        }
      }
//      this->driver.setJointPosition(cmd_q);
//      this->driver.setJointVelocity(cmd_qd);
//      this->driver.setJointAcceleration(cmd_qdd);
    }
  }
  if (isJointsWithSafetyLimit(act_q)) {  //位置超限打印
    flag += 2;
    if (protect) {
      for (int j = 0; j < this->dof; ++j) {
        if (this->joint_pos_limit_flag[j]) {
          if (act_q[j] > this->upper_pos_limit[j]) {
            // ::std::cout << RED << " Joint[" << j << "] act_pos: " << act_q[j] << " is out of Algorithm UPPER limit: " << this->upper_pos_limit[j] << RESET << std::endl;
          }
          if (act_q[j] < this->lower_pos_limit[j]) {
            // ::std::cout << RED << " Joint[" << j << "] act_pos: " << act_q[j] << " is out of Algorithm LOWER limit: " << this->lower_pos_limit[j] << RESET << std::endl;
          }
        }
      }
    }
  }
  return flag;
}

bool JointsSecurity::jointsVelAccLimitProtection(::ul::std17::RobotCommand& cmd) {
//  this->cmd_q = this->driver.getCommandJointPosition(); // TODO: 待完善双缓冲机制
//  this->cmd_qd = this->driver.getCommandJointVelocity();
//  this->cmd_q_last = this->driver.getLastCommandJointPosition();
//  this->cmd_qd_last = this->driver.getLastCommandJointVelocity();

  bool pos_threshold_flag = false;
  for (int j = 0; j < this->dof; j++) {
    if ((abs(cmd_q[j] - cmd_q_last[j]) > this->pos_threshold || abs(cmd_qd[j] - cmd_qd_last[j]) > 0.1) && cmd != ::ul::std17::RobotCommand::TEACH) {
      pos_threshold_flag = true;
      std::cout << RED << "[Error] Joint[" << j << "] velocity or acceleration exceeds the limit!" << RESET << std::endl;
      std::cout << RED << "1. Please check if the robot is in a singular position or at a joint limit!" << RESET << std::endl;
      std::cout << RED << "2. Please check for sudden changes in the input position!" << RESET << std::endl;
      std::cout << RED << "run_cmd: " << static_cast<int>(cmd) << RESET << std::endl;
      std::cout << RED << "cmd_q: " << cmd_q.transpose() << RESET << std::endl;
      std::cout << RED << "cmd_q_last: " << cmd_q_last.transpose() << RESET << std::endl;
      std::cout << RED << "cmd_qd: " << cmd_qd.transpose() << RESET << std::endl;
      std::cout << RED << "cmd_qd_last: " << cmd_qd_last.transpose() << RESET << std::endl;
      break;
    }
  }
  
  if (pos_threshold_flag == true && cmd != ::ul::std17::RobotCommand::PROTECTED) {  // 速度超限且未处于保护状态
    cmd_qdd.setZero();
    cmd_qd = cmd_qd_last;
    cmd_q = cmd_q_last + this->dt * cmd_qd_last;
//    this->driver.setJointPosition(cmd_q);
//    this->driver.setJointVelocity(cmd_qd);
//    this->driver.setJointAcceleration(cmd_qdd);
    return true;
  } else {
    return false;
  }
}

}  // namespace controller
}  // namespace ul