/*
 * @Description:  
 * @Author: AN Hao, YUAN Hanqing
 * @Date: 2025-03-11 17:40:28
 * @LastEditors: YUAN Hanqing
 * @LastEditTime: 2025-03-18 09:37:51
 * @FilePath: /uplimlibrary/src/ul/controller/JointsSecurity.h
 */

#ifndef UL_SRC_UL_CONTROLLER_JOINTSSECURITY_H_
#define UL_SRC_UL_CONTROLLER_JOINTSSECURITY_H_

#include <ul/hal/Coach.h>
#include <ul/std/common.h>
#include <vector>

#include <iostream>


namespace ul {
namespace controller {
class JointsSecurity {
 public:

  JointsSecurity(const ::std::size_t& dof);

  ~JointsSecurity();

  void setJointPositionLimit(const ::ul::math::Vector& lower_limit, const ::ul::math::Vector& upper_limit);

  void getJointPositionLimit(::ul::math::Vector& lower_limit, ::ul::math::Vector& upper_limit) const;

  void setJointPositionThreshold(const double pos_threshold);

  void getJointPositionThreshold(double& pos_threshold) const;

  bool boundNormalize(const double &lb, const double &ub, const double &epsilon, const double &x0, double &x);

  bool isJointsWithSafetyLimit(const ::ul::math::Vector& q);

  int jointsPosLimitProtection(bool protect);

  bool jointsVelAccLimitProtection(::ul::std17::RobotCommand& cmd);

 private:
  ::ul::math::Vector cmd_q, cmd_qd, cmd_qdd, cmd_qddd;

  ::ul::math::Vector act_q, act_qd, act_qdd, act_qddd;

  ::ul::math::Vector cmd_q_last, cmd_qd_last; 

  ::std::vector<::std::uint8_t> act_mode;

  ::std::size_t dof;

  ::ul::math::Real dt;

  ::ul::math::Real pos_threshold;

  ::ul::math::Vector lower_pos_limit, upper_pos_limit;

  ::std::vector<bool> joint_pos_limit_flag;  // 用来存储是否超出关节位置限制，1为超出关节限制

  ::std::vector<bool> joint_vel_limit_flag;  // 用来存储是否超出关节速度限制，1为超出关节限制

  ::std::vector<bool> joint_tau_limit_flag;  // 用来存储是否超出关节力矩限制，1为超出关节限制
};
}  // namespace controller
}  // namespace ul

#endif  // UL_SRC_UL_CONTROLLER_JOINTSSECURITY_H_
