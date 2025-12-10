/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-1-14
 * @Version       : 2.0.1
 * @File          : Controller.cpp
 ******************************************************************************
 */

#include "Controller.h"

namespace ul {
namespace controller {

Controller::Controller(const ::std::string& config_path) : config_(YAML::LoadFile(config_path)),  // 先初始化config_
                                                           robot_([this]{
                                                             return this->config_["URDF_FILE"].as<::std::string>();
                                                           }()),
                                                           driver_(robot_.getDof(), 0.001),
                                                           dof_(robot_.getDof()),
                                                           run_cmd_(::ul::std17::RobotCommand::NO_CMD),
                                                           run_mode_(::ul::std17::RobotRunMode::CSP),
                                                           cartesian_trajectory_length_(4),
                                                           interaction_control_(driver_, robot_, robot_.getDof()),
                                                           algorithm_idx_(robot_.getDof()),
                                                           cmd_server_("tcp://*:5555"),
                                                           double_buffer_() { readConfig(); createRobot(); calculateDof();}

Controller::~Controller() {}

void Controller::calculateDof() {
  size_t dof = 0;
  for (auto &body : robot_cfg_->bodies) {
    dof += body->getDof();
  }
  this->dof_ = dof;
}

::std::size_t Controller::getDof() { return this->dof_; }
//
//void Controller::setFTSensor(::ul::math::Vector& ft_l, ::ul::math::Vector& ft_r) {
//  interaction_control_.setFTSensor(ft_l, ft_r);
//}
//
//void Controller::getFTSensor(::ul::math::Vector& ft_l, ::ul::math::Vector& ft_r) {
//  interaction_control_.getFTSensor(ft_l, ft_r);
//}
//
::std::vector<::std::string> Controller::getJointName() { return this->robot_.getJointName(); }
//
void Controller::setJointPositionLimit(const ::ul::math::Vector& lower_limit, const ::ul::math::Vector& upper_limit) {
//  this->joints_security_.setJointPositionLimit(lower_limit, upper_limit);  // TODO: body中设置
  this->interaction_control_.setJointPositionLimit(lower_limit, upper_limit);
}

//void Controller::getJointPositionLimit(::ul::math::Vector& lower_limit, ::ul::math::Vector& upper_limit) const {
//  this->joints_security_.getJointPositionLimit(lower_limit, upper_limit);
//}
//
void Controller::setJointPositionThreshold(const double pos_threshold) {
//  this->joints_security_.setJointPositionThreshold(pos_threshold);  // TODO: body中设置
}

//void Controller::getJointPositionThreshold(double& pos_threshold) const {
//  this->joints_security_.getJointPositionThreshold(pos_threshold);
//}
//
void Controller::setJointFrictionParas(const double s_k, const ::ul::math::Vector& f_a, const ::ul::math::Vector& f_b, const ::ul::math::Vector& f_c) {
  this->robot_.setJointFrictionParas(s_k, f_a, f_b, f_c);
}

//bool Controller::setControllerParas(const char *file_name) {
//  return this->interaction_control_.setControllerParas(file_name, true);
//}
//
//bool Controller::setControllerPara(::ul::controller::ParasList &para_label, ::ul::math::Vector &para) {
//  return this->interaction_control_.setControllerPara(para_label, para, true);
//}
//
//bool Controller::getControllerPara(::ul::controller::ParasList &para_label, ::ul::math::Vector &para) {
//  return this->interaction_control_.getControllerPara(para_label, para);
//}
//
//bool Controller::setControllerMode(::ul::controller::ControllerMode &mode) {
//  if ((::ul::std17::RobotCommand::NO_CMD != run_cmd_) && (::ul::std17::RobotCommand::PROTECTED != run_cmd_)) {
//    ::std::cout << RED << "[ERROR setControllerMode] Never switch the mode during robot motion!" << RESET << ::std::endl;
//    return false;
//  }
//  ::std::vector<uint8_t> mode_operation(dof_, 8);
//  ::std::unique_lock<::std::mutex> lock(this->mtx_);
//  this->controller_mode_ = mode;
//  if (mode == ::ul::controller::ControllerMode::POSITION_CSP) {
//    std::cout << YELLOW << "Switch to CSP-Based Trajectory Tracking!" << RESET << std::endl;
//  } else if (mode == ::ul::controller::ControllerMode::POSITION_CST) {
//    std::fill_n(mode_operation.begin(), joints_num_[0] + joints_num_[1], 10); // 双臂切换到CST
//    std::cout << YELLOW << "Switch to CST-Based Trajectory Tracking!" << RESET << std::endl;
//  } else if (mode == ::ul::controller::ControllerMode::IMPEDANCE) {
//    std::fill_n(mode_operation.begin(), joints_num_[0] + joints_num_[1], 10); // 双臂切换到CST
//    std::cout << YELLOW << "Switch to Impedance Control!" << RESET << std::endl;
//  }
//  this->driver_.setJointModeOperation(mode_operation);
//  // 清控力矩滤波器
//  return true;
//}
//
//bool Controller::getSafetyLock() {
//  return this->emergency_stop_flag_;
//}
//
//bool Controller::releaseSafetyLock() {
//  if (emergency_stop_flag_) {
//    ::std::unique_lock<::std::mutex> lock(this->mtx_);
//    sendCurrentJointPosition();
//    ::std::vector<uint8_t> mode_operation(dof_, 8);
//    if(this->controller_mode_ != ::ul::controller::ControllerMode::POSITION_CSP) {
//      std::fill_n(mode_operation.begin(), joints_num_[0] + joints_num_[1], 10); // 双臂切换到CST
//    }
//    this->driver_.setJointModeOperation(mode_operation);
//
//    this->protect_flag_vel_acc_ = false;
//    this->emergency_stop_flag_ = false;
//    this->is_collsion_flag_ = false;
//    this->run_cmd_ = ::ul::std17::RobotCommand::NO_CMD;
//    ::std::cout << YELLOW <<  "[WARN] Protection mode has been deactivated on the robot!" << RESET << std::endl;
//    return true;
//  } else {
//    ::std::cout << YELLOW <<  "[ERROR] Protection mode remains deactivated on the robot!" << RESET << std::endl;
//    return false;
//  }
//}
//
//::std::vector<::ul::math::Vector6> Controller::getCommandCartesianAcceleration() { return cartesian_motion_.getCommandCartesianAcceleration(); }
//
//::std::vector<::ul::math::Vector6> Controller::getCommandCartesianPosition() { return cartesian_motion_.getCommandCartesianPosition(); }
//
//::std::vector<::ul::math::Vector6> Controller::getCommandCartesianVelocity() { return cartesian_motion_.getCommandCartesianVelocity(); }
//
//::ul::math::Real Controller::getCommandDownTime() { return this->driver_.getCommandDownTime(); }
//
::ul::math::Vector Controller::getCommandJointAcceleration() { return this->driver_.getCommandJointAcceleration(); }
//
::std::vector<::std::uint16_t> Controller::getCommandJointCtrlwd() { return this->driver_.getCommandJointCtrlwd(); }
//
::std::vector<::std::uint8_t> Controller::getCommandJointModeOperation() { return this->driver_.getCommandJointModeOperation(); }
//
::ul::math::Vector Controller::getCommandJointOffsetTorque() { return this->driver_.getCommandJointOffsetTorque(); }

 ::ul::math::Vector Controller::getCommandJointOffsetVelocity() { return this->driver_.getCommandJointVelocity().array() * Eigen::Map<const Eigen::VectorXi>(offset_vel_on_.data(), offset_vel_on_.size()).cast<double>().array(); }

//::ul::math::Vector Controller::getCommandJointOffsetVelocity() {
//    const auto& velocities = this->driver_.getCommandJointVelocity();
//    const auto& mode_ops = this->driver_.getCommandJointModeOperation();
//    const auto& offsets = Eigen::Map<const Eigen::VectorXi>(offset_vel_on_.data(), offset_vel_on_.size()).cast<double>();
//
//    ::ul::math::Vector result = velocities;
//    for(int i = 0; i < result.size(); ++i) {
//        result[i] *= (mode_ops[i] == 8) ? offsets[i] : 0.0;
//    }
//    return result;
//}
//
::ul::math::Vector Controller::getCommandJointPosition() { return this->driver_.getCommandJointPosition(); }
//
::ul::math::Vector Controller::getCommandJointVelocity() { return this->driver_.getCommandJointVelocity(); }
//
::ul::math::Vector Controller::getCommandJointTorque() { return this->driver_.getCommandJointTorque(); }
//
//::std::vector<::ul::math::Vector6> Controller::getEEAcc(const ::ul::math::Vector& q, const ::ul::math::Vector& qd, const ::ul::math::Vector& qdd) {
//  assert(q.size() == this->dof_ && qd.size() == this->dof_ && qdd.size() == this->dof_);
//  ::std::vector<::ul::math::Vector6> ee_accelerations(2);
//  {
//    ::std::unique_lock<::std::mutex> lock(mtx_);
//    this->robot_.setPosition(q);
//    this->robot_.setSpeed(qd);
//    this->robot_.setAcceleration(qdd);
//    for (int i = 0; i < this->cartesian_config_.ee_name.size(); i++) {
//      ee_accelerations[i] = this->robot_.getOperationalAcceleration(this->cartesian_config_.ee_idx[i]);
//    }
//  }
//  return ee_accelerations;
//}
//
//::std::vector<::ul::math::Vector6> Controller::getEEVel() {
//  return this->getEEVel(driver_.getJointPosition(), driver_.getJointVelocity());
//}
//
//::std::vector<::ul::math::Vector6> Controller::getEEVel(const ::ul::math::Vector& q, const ::ul::math::Vector& qd) {
//  assert(q.size() == this->dof_ && qd.size() == this->dof_);
//  ::std::vector<::ul::math::Vector6> ee_velocities(2);
//  {
//    ::std::unique_lock<::std::mutex> lock(mtx_);
//    this->robot_.setPosition(q);
//    this->robot_.setSpeed(qd);
//    for (int i = 0; i < this->cartesian_config_.ee_name.size(); i++) {
//      ee_velocities[i] = this->robot_.getOperationalVelocity(this->cartesian_config_.ee_idx[i]);
//    }
//  }
//  return ee_velocities;
//}
//
//::std::vector<::ul::math::Vector6> Controller::getForwardKinematics() {
//  return this->getForwardKinematics(driver_.getJointPosition());
//}
//
//::std::vector<::ul::math::Vector6> Controller::getForwardKinematics(const ::ul::math::Vector& q) {
//  assert(q.size() == this->dof_);
//  ::std::vector<::ul::math::Vector6> ee_poses(2);
//  {
//    ::std::unique_lock<::std::mutex> lock(mtx_);
//    this->robot_.setPosition(q);
//    for (int i = 0; i < this->cartesian_config_.ee_name.size(); i++) {
//      ::ul::math::Transform op_trans = this->robot_.getOperationalTransform(this->cartesian_config_.ee_idx[i]);
//      ::ul::math::Transform2Pose(op_trans, ee_poses[i]);
//    }
//  }
//  return ee_poses;
//}
//
//int Controller::getInverseKinematics(const ::std::vector<::ul::math::Vector6>& pose,
//                                      const ::std::vector<::ul::math::Real>& q7,
//                                      ::std::vector<::std::vector<Eigen::VectorXd>> &q,
//                                      ::std::vector<::std::vector<double>> &phi) {
//  std::lock_guard<std::mutex> lock(mtx_);
//  return cartesian_motion_.getInverseKinematics(pose, q7, q, phi);
//}
//
//::ul::math::Vector Controller::getGravityTorque() {
//  return this->getGravityTorque(this->driver_.getJointPosition());
//}
//
//::ul::math::Vector Controller::getGravityTorque(const ::ul::math::Vector& q) {
//  assert(q.size() == this->dof_);
//  ::std::unique_lock<::std::mutex> lock(mtx_);
//  // 设置机器人位置
//  this->robot_.setPosition(q);
//  // 计算重力矩
//  this->robot_.calculateGravity();
//  return this->robot_.getGravity();
//}
//
//::std::vector<::ul::math::Matrix> Controller::getJacobian() {
//  return this->getJacobian(driver_.getJointPosition());
//}
//
//::std::vector<::ul::math::Matrix> Controller::getJacobian(const ::ul::math::Vector& q) {
//  ::std::vector<::ul::math::Matrix> J(2, ::ul::math::Matrix::Zero(6, this->dof_));
//  if (q.size() != this->dof_) {
//    ::std::cout << "[ERROR getJacobian] Input q size " << q.size() << " != DOF " << this->dof_ << ::std::endl;
//    return J;
//  }
//  {
//    std::lock_guard<std::mutex> lock(mtx_);
//    this->robot_.setPosition(q);
//    for (int i = 0; i < 2; ++i) {
//      if (this->robot_.existFrame(this->cartesian_config_.ee_name[i])) {
//        this->robot_.setOperationalFrameIndex(this->cartesian_config_.ee_idx[i]);
//        this->robot_.setOperationalFrameName(this->cartesian_config_.ee_name[i]);
//        this->robot_.calculateJacobian(1);
//        J[i] = this->robot_.getJacobian().block(0, this->cartesian_config_.joint_idx_first[i] - 1, 6, this->cartesian_config_.joint_num[i]);
//      } else {
//        ::std::cout << "[ERROR getJacobian] Frame " << this->cartesian_config_.ee_name[i] << " does not exist!" << ::std::endl;
//        return J;
//      }
//    }
//  }
//  return J;
//}
//
void Controller::getJointMode(::std::vector<::std::uint8_t>& mode) { this->driver_.getJointMode(mode); }

void Controller::getJointPosition(::ul::math::Vector& q) { this->driver_.getJointPosition(q); }

void Controller::getJointVelocity(::ul::math::Vector& qd) { this->driver_.getJointVelocity(qd); }

void Controller::getJointStatusWord(::std::vector<::std::uint16_t>& statuswd) { this->driver_.getJointStatusWord(statuswd); }

void Controller::getJointTorque(::ul::math::Vector& tau) { this->driver_.getJointTorque(tau); }

void Controller::getJointUploadTime(::ul::math::Real& upTime) { this->driver_.getJointUploadTime(upTime); }

//::ul::math::Vector Controller::getLastCommandJointPosition() { return this->driver_.getLastCommandJointPosition(); }
//
//::ul::math::Vector Controller::getLastCommandJointVelocity() { return this->driver_.getLastCommandJointVelocity(); }
//
const ::std::string& Controller::getRobotName() { return this->robot_.getName(); }
//
//::ul::std17::RobotCommand Controller::getRobotRunCommand() const { return this->run_cmd_; }
//
//int Controller::getTrajectoryCurrentStep() const { return this->trajectory_planning_step_; }
//
//int Controller::getTrajectoryLength() const { return this->trajectory_planning_length_;}
//
//::std::vector<uint8_t> Controller::getJointMode() const { return this->driver_.getJointMode(); }
//
//::ul::math::Vector Controller::getJointPosition() const { return this->driver_.getJointPosition();}
//
//::ul::math::Vector Controller::getJointTorque() const { return this->driver_.getJointTorque(); }
//
//::ul::math::Vector Controller::getJointVelocity() const { return this->driver_.getJointVelocity(); }
//
//void Controller::fsm(::ul::std17::RobotCommand& cmd, ::ul::std17::RobotRunMode& mode, bool& first, const int& length, bool& is_complete, int& step) {
//  switch (cmd) {
//    case ::ul::std17::RobotCommand::NO_CMD:
//      is_complete = false;
//      step = 0;
//      if (true == first) {
//        sendCurrentJointPosition();
//        ::std::cout << "Initial actual joint position: " << this->driver_.getJointPosition().transpose() << ::std::endl;
//        first = false;
//      } else {
//        ::ul::math::Vector temp(robot_.getDof());
//        temp = (driver_.getCommandJointPosition()  - driver_.getLastCommandJointPosition())/driver_.getUpdateRate();
//        this->driver_.setJointVelocity(temp);
//        temp = (driver_.getCommandJointVelocity() - driver_.getLastCommandJointVelocity())/driver_.getUpdateRate();
//        this->driver_.setJointAcceleration(temp);
//      }
//      break;
//    case ::ul::std17::RobotCommand::MOVEJ:
//      if (is_complete) {
//        if (step < length) {
////          this->trajectory_generated_.moveJ_trajectory(step);
//          this->joints_motion_.moveJ_trajectory(step);
//          step++;
//        } else {
//          resetState(cmd, is_complete, step, "MOVEJ");
//          cond_.notify_one();
//        }
//      }
//      break;
//    case ::ul::std17::RobotCommand::MOVEJ_PATH:
//      if (is_complete) {
//        if (step < length) {
//          joints_motion_.moveJ_path_trajectory(step);
//          step++;
//        } else {
//          resetState(cmd, is_complete, step, "MOVEJ_PATH");
//          cond_.notify_one();
//        }
//      }
//      break;
//    case std17::RobotCommand::MOVEL_NULLSPACE:
//      if (is_complete) {
//        if (step < length) {
//          cartesian_motion_.moveL_trajectory_nullspace(step);
//          step++;
//        } else {
//          resetState(cmd, is_complete, step, "MOVEL_NULLSPACE");
//          cond_.notify_one();
//        }
//      }
//      break;
//    case std17::RobotCommand::MOVEL:
//      if (is_complete) {
//        if (step < length) {
//          this->robot_.setPosition(this->driver_.getLastCommandJointPosition());
//          cartesian_motion_.moveL_trajectory(step);
//          step++;
//        } else {
//          resetState(cmd, is_complete, step, "MOVEL");
//          cond_.notify_one();
//        }
//      }
//      break;
//    case std17::RobotCommand::MOVEL_PATH:
//      if (is_complete) {
//        if (step < length) {
//          this->robot_.setPosition(this->driver_.getLastCommandJointPosition());
//          cartesian_motion_.moveL_path_trajectory(step);
//          step++;
//        } else {
//          resetState(cmd, is_complete, step, "MOVEL_PATH");
//          cond_.notify_one();
//        }
//      }
//      break;
//    case std17::RobotCommand::SPEEDJ:
//      if (is_complete) {
//        this->joints_motion_.speedJ_trajectory(step);
//        step++;
//        if (step == length) {
//          // resetState(cmd, is_complete, step, "SPEEDJ");
//          cond_.notify_one();
//        }
//      }
//      break;
//    case std17::RobotCommand::SPEEDL:
//      if (is_complete) {
//        this->robot_.setPosition(this->driver_.getLastCommandJointPosition());
//        this->cartesian_motion_.speedL_trajectory(step);
//        step++;
//        if (step == length) {
//          // resetState(cmd, is_complete, step, "SPEEDL");
//          cond_.notify_one();
//        }
//      }
//      break;
//    case std17::RobotCommand::SPEED_STOP:
//      if (is_complete) {
//        if (step < length) {
//          this->joints_motion_.speed_stop_trajectory(step);
//          step++;
//        } else {
//          resetState(cmd, is_complete, step, "SPEED_STOP");
//          cond_.notify_one();
//        }
//      }
//      break;
//   case std17::RobotCommand::MOVEFOURIER:
//      break;
//    case std17::RobotCommand::MOVECSVFILE:
//      if (is_complete) {
//        if (step < length) {
//          this->joints_motion_.csv_trajectory(step);
//          step++;
//        } else {
//          resetState(cmd, is_complete, step, "MOVECSVFILE");
//          cond_.notify_one();
//        }
//      }
//      break;
//    case std17::RobotCommand::MOVEJ_SPLINE:
//      if (is_complete) {
//        if (step < length) {
//          this->joints_motion_.moveJ_spline_trajectory(step);
//          step++;
//        } else {
//          resetState(cmd, is_complete, step, "MOVEJ_SPLINE");
//          cond_.notify_one();
//        }
//      }
//      break;
//    case std17::RobotCommand::TEACH:
//      this->interaction_control_.teachTorqueGenerate();
//      break;
//    case std17::RobotCommand::SERVOJ:
//      if (is_complete) {
//        if (this->joints_motion_.servoJ_trajectoryGenerated(step)) {
//          resetState(cmd, is_complete, step, "SERVOJ");
//        }
//        step++;
//      }
//      break;
//    case std17::RobotCommand::SERVOL:
//      if (is_complete) {
//        this->robot_.setPosition(this->driver_.getLastCommandJointPosition());
//        if (this->cartesian_motion_.servoL_trajectoryGenerated(step)) {
//          resetState(cmd, is_complete, step, "SERVOL");
//        }
//        step++;
//      }
//      break;
//    case std17::RobotCommand::PROTECTED:
//      this->interaction_control_.impedanceController_protected();
//      break;
//  }
//};
//
//bool Controller::goBack(::std::vector<::ul::math::Vector> q_home) {
//  ::std::vector<::ul::math::Vector> q_temp(2), singular_qd(2);
//  ::ul::math::Real damping = 0.05;
//  int idx = 0;
//  ::std::vector<bool> isSingular(2);
//  ::std::vector<::ul::math::Matrix> J(2), J_temp(2);
//  double delta_q = 0.1;
//  {
//    std::lock_guard<std::mutex> lock(mtx_);
//    singular_qd = this->cartesian_motion_.getSingularJointVelocity();
//    this->robot_.setPosition(this->driver_.getLastCommandJointPosition());
//    for (int i = 0; i < 2; ++i) {
//      if (this->robot_.existFrame(this->cartesian_config_.ee_name[i])) {
//        this->robot_.setOperationalFrameIndex(this->cartesian_config_.ee_idx[i]);
//        this->robot_.setOperationalFrameName(this->cartesian_config_.ee_name[i]);
//        this->robot_.calculateJacobian(1);
//        J_temp[i] = this->robot_.getJacobian().block(0, this->cartesian_config_.joint_idx_first[i] - 1, 6, this->cartesian_config_.joint_num[i]);
//        if (this->cartesian_config_.joint_num[i] < 6){
//          J[i].resize(this->cartesian_config_.joint_num[i], this->cartesian_config_.joint_num[i]);
//          for (int j = 0; j < this->cartesian_config_.joint_num[i]; ++j) {
//            J[i].row(j) = J_temp[i].row(this->cartesian_config_.cartesian_select[i][j]);
//          }
//        } else {
//          J[i] = J_temp[i];
//        }
//        if (this->robot_.isSingular(J[i], damping)) {
//          isSingular[i] = true;
//          q_temp[i] = this->driver_.getLastCommandJointPosition().segment(idx, this->cartesian_config_.joint_num[i]);
//          if (singular_qd[i].cwiseAbs().maxCoeff() > 0.001) {  //在奇异位置，且存在返回速度
//              q_temp[i] -= delta_q*singular_qd[i];
//              // ::std::cout << arm_name[i] << " 在奇异位置，且存在返回速度" << singular_qd[i].transpose() << std::endl;
//          } else { //在奇异位置，且不存在返回速度
//              // ::std::cout << arm_name[i] << " 在奇异位置，且不存在返回速度" << std::endl;
//              // q_home <-- q_temp
//              ::ul::math::Vector delta = q_home[i] - q_temp[i];
//              double max_abs = delta.cwiseAbs().maxCoeff();
//              ::ul::math::Vector step;
//              if (max_abs > 0.01) {
//                  step = (delta/max_abs) * delta_q;
//              } else {
//                  step.resize(this->cartesian_config_.joint_num[i]);
//                  step.setZero();
//              }
//              q_temp[i] += (delta.array().abs() < delta_q).select(delta, step);
//          }
//        } else { // 不在奇异位置
//          ::std::cout << this->cartesian_config_.arm_type[i] << " 不在奇异位置" << std::endl;
//          isSingular[i] = false;
//        }
//      } else {
//        ::std::cout << "[ERROR goBack] Frame " << this->cartesian_config_.ee_name[i] << " does not exist!" << ::std::endl;
//        return false;
//      }
//      idx += this->cartesian_config_.joint_num[i];
//    }
//  }
//
//  idx = 0;
//  if (isSingular[0] || isSingular[1]) {
//    ::ul::math::Vector q_target = this->driver_.getLastCommandJointPosition();
//    for (int i = 0; i < 2; ++i) {
//      if (isSingular[i]) {
//        q_target.segment(idx, this->cartesian_config_.joint_num[i]) = q_temp[i];
//      }
//      idx += this->cartesian_config_.joint_num[i];
//    }
//    if (!moveJ(q_target, 0.1, 0.2, false)) return false;
//  } else {
//    std::cout << RED << "[ERROR goBack] The robot is not in a singularity!" << RESET << std::endl;
//    return false;
//  }
//
//  return true;
//}
//
//int Controller::isSingular() {
//  return isSingular(this->driver_.getLastCommandJointPosition());
//}
//
//int Controller::isSingular(const ::ul::math::Vector& q) {
//  if (q.size() != this->dof_) {
//    ::std::cout << "[ERROR isSingular] Input q size " << q.size() << " != DOF " << this->dof_ << ::std::endl;
//    return -1;
//  }
//  ::ul::math::Real damping = 0.05;
//  int isSingular = 0;
//  ::std::vector<::ul::math::Matrix> J(2, ::ul::math::Matrix::Zero(6, this->dof_));
//  {
//    std::lock_guard<std::mutex> lock(mtx_);
//    this->robot_.setPosition(q);
//    for (int i = 0; i < 2; ++i) {
//      if (this->robot_.existFrame(this->cartesian_config_.ee_name[i])) {
//        this->robot_.setOperationalFrameIndex(this->cartesian_config_.ee_idx[i]);
//        this->robot_.setOperationalFrameName(this->cartesian_config_.ee_name[i]);
//        this->robot_.calculateJacobian(1);
//        J[i] = this->robot_.getJacobian().block(0, this->cartesian_config_.joint_idx_first[i] - 1, 6, this->cartesian_config_.joint_num[i]);
//        if (this->robot_.isSingular(J[i], damping)) {
//          if(i == 0) {
//            isSingular += 1;
//          } else if (i == 1) {
//            isSingular += 2;
//          }
//        }
//      } else {
//        ::std::cout << "[ERROR isSingular] Frame " << this->cartesian_config_.ee_name[i] << " does not exist!" << ::std::endl;
//        return 0;
//      }
//    }
//  }
//  return isSingular;
//}
//
//bool Controller::moveJ(const ::ul::math::Vector& q, double speed, double acceleration, bool asynchronous) {
//  if (::ul::std17::RobotCommand::TEACH == run_cmd_) {
//    ::std::cout << RED << "[ERROR moveJ] Please exit teach mode!" << RESET << ::std::endl;
//    return false;
//  }
//
//  if (init_pos_limit) {
//    ::std::cout << RED << "[ERROR moveJ] Please wait for the joints to return to position!" << RESET << ::std::endl;
//    return false;
//  }
//  ::std::unique_lock<std::mutex> lock(mtx_);
//  if (!joints_motion_.moveJ(q, speed, acceleration)) return false;
//
//  // 设置规划结束后的运动状态
//  run_cmd_ = ::ul::std17::RobotCommand::MOVEJ;
//  trajectory_planning_length_ = joints_motion_.getTrajectoryLength();
//  trajectory_planning_complete_ = true;
//  trajectory_planning_step_ = 0;
//
//  // 阻塞等待
//  this->unblockThread(asynchronous, lock);
//
//  return true;
//}
//
//// 指定末端位姿，以关节空间的方式运动
//bool Controller::moveJ(const ::std::vector<math::Vector6> &pose, const ::std::vector<math::Real>& q7, double speed, double acceleration, bool asynchronous) {
//  // 第一个vector：0:左手关节角组成的vector  1:右手关节角组成的vector
//  // 第二个vector: 每个手对应的关节角组数
//  ::std::vector<::std::vector<Eigen::VectorXd>> q_temp;
//  ::std::vector<::std::vector<double>> phi;
//  if (!this->getInverseKinematics(pose, q7, q_temp, phi)) {
//    ::std::cout << RED << "[ERROR moveJ_pose] Target pose can not be reached!" << RESET << ::std::endl;
//    return false;
//  }
//
//  math::Vector cmd_last_q = this->getLastCommandJointPosition();
//
//  // 依据关节误差二范数最小来选择期望关节角
//  std::vector<math::Vector> q_target(q_temp.size());
//  for (int j = 0; j < q_temp.size(); ++j) {
//    math::Real min_error_norm = 10000;  // 换手时重置最小范数为10000
//    for (int i = 0; i < q_temp[j].size(); ++i) {
//      math::Real current_error_norm = (cmd_last_q.segment<7>(7*j) - q_temp[j][i]).norm();
//      if (current_error_norm < min_error_norm) {
//        min_error_norm = current_error_norm;
//        q_target[j] = q_temp[j][i];
//      }
//    }
//  }
//
//  // 拼接全身关节角
//  math::Vector target_q(this->getDof());
//  target_q.segment<7>(0) = q_target[0];
//  target_q.segment<7>(7) = q_target[1];
//  for (int j = 14; j < this->getDof(); ++j) {
//    target_q[j] = cmd_last_q[j];
//  }
//  return this->moveJ(target_q, speed, acceleration, asynchronous);
//}
//
//bool Controller::moveJ(const math::Vector& q, const math::Vector& qd, double freq, bool asynchronous) {
//  ::std::unique_lock<std::mutex> lock(mtx_);
//  if (::ul::std17::RobotCommand::TEACH == run_cmd_) {
//    ::std::cout << RED << "[ERROR moveJ_Spline] Please exit teach mode!" << RESET << ::std::endl;
//    return false;
//  }
//  if (init_pos_limit) {
//    ::std::cout << RED << "[ERROR moveJ_Spline] Please wait for the joints to return to position!" << RESET << ::std::endl;
//    return false;
//  }
//  // std::cout << BLUE << "q: " << q.transpose() << RESET << std::endl;
//  // std::cout << BLUE << "qd: " << qd.transpose() << RESET << std::endl;
//  joints_motion_.moveJ_Spline(q, qd, freq);
//
//  // 设置规划结束后的运动状态
//  run_cmd_ = ::ul::std17::RobotCommand::MOVEJ_SPLINE;
//  ::ul::math::Real time = 1.0/(freq);
//  time = round(time * 1000) / 1000;
//  trajectory_planning_length_ = ceil(time/driver_.getUpdateRate());
//
//  trajectory_planning_complete_ = true;
//  trajectory_planning_step_ = 0;
//
//  // 阻塞等待
//  this->unblockThread(asynchronous, lock);
//
//  return true;
//}
//
//bool Controller::moveJ(const std::vector<std::vector<double>>& path, const double& time, bool asynchronous) {
//  if (::ul::std17::RobotCommand::TEACH == run_cmd_) {
//    ::std::cout << RED << "[ERROR moveJ_Path] Please exit teach mode!" << RESET << ::std::endl;
//    return false;
//  }
//  if (init_pos_limit) {
//    ::std::cout << RED << "[ERROR moveJ_Path] Please wait for the joints to return to position!" << RESET << ::std::endl;
//    return false;
//  }
//  if (::ul::std17::RobotCommand::NO_CMD != run_cmd_) {
//    ::std::cout << RED << "[ERROR moveJ_Path] Please wait for the robot to stop!" << RESET << ::std::endl;
//    return false;
//  }
//  joints_motion_.moveJ(path, time);
//  ::std::unique_lock<::std::mutex> lock(mtx_);
//  // 设置规划结束后的运动状态
//  run_cmd_ = ::ul::std17::RobotCommand::MOVEJ_PATH;
//  trajectory_planning_length_ = ceil(time/driver_.getUpdateRate());
//  trajectory_planning_complete_ = true;
//  trajectory_planning_step_ = 0;
//
//  // 阻塞等待
//  this->unblockThread(asynchronous, lock);
//
//  return true;
//}
//
//bool Controller::moveJ(const std::vector<std::vector<double>>& path, const ::std::vector<double>& time, bool asynchronous) {
//  if (::ul::std17::RobotCommand::TEACH == run_cmd_) {
//    ::std::cout << RED << "[ERROR moveJ_Path] Please exit teach mode!" << RESET << ::std::endl;
//    return false;
//  }
//  if (init_pos_limit) {
//    ::std::cout << RED << "[ERROR moveJ_Path] Please wait for the joints to return to position!" << RESET << ::std::endl;
//    return false;
//  }
//  if (::ul::std17::RobotCommand::NO_CMD != run_cmd_) {
//    ::std::cout << RED << "[ERROR moveJ_Path] Please wait for the robot to stop!" << RESET << ::std::endl;
//    return false;
//  }
//  joints_motion_.moveJ(path, time);
//
//  ::std::unique_lock<::std::mutex> lock(mtx_);
//  // 设置规划结束后的运动状态
//  run_cmd_ = ::ul::std17::RobotCommand::MOVEJ_PATH;
//
//  trajectory_planning_length_ = ceil(time.back()/driver_.getUpdateRate());
//  trajectory_planning_complete_ = true;
//  trajectory_planning_step_ = 0;
//
//  // 阻塞等待
//  this->unblockThread(asynchronous, lock);
//
//  return true;
//}
//
//
//bool Controller::moveL(const ::std::vector<::ul::math::Vector6> &pose, const ::ul::math::Vector &ref_q, const double& speed, const double& acceleration, const bool& asynchronous) {
//  ::std::unique_lock<::std::mutex> lock(mtx_);
//  cartesian_motion_.moveLNullSpace(pose, ref_q, speed, acceleration);
//
//  // 设置规划结束后的运动状态
//  run_cmd_ = ::ul::std17::RobotCommand::MOVEL_NULLSPACE;
//  trajectory_planning_length_ = cartesian_motion_.getTrajectoryLength();
//  trajectory_planning_complete_ = true;
//  trajectory_planning_step_ = 0;
//
//  // 阻塞等待
//  this->unblockThread(asynchronous, lock);
//
//  return true;
//}
//
//bool Controller::moveL(const ::std::vector<::ul::math::Vector6> &pose, const double& speed, const double& acceleration, const bool& asynchronous) {
//  // ::std::cout << "pose[0]: " << pose[0].transpose() << std::endl;
//  // ::std::cout << "pose[1]: " << pose[1].transpose() << std::endl;
//  ::std::unique_lock<::std::mutex> lock(mtx_);
//  if (::ul::std17::RobotCommand::TEACH == run_cmd_) {
//    ::std::cout << RED << "[ERROR moveL] Please exit teach mode!" << RESET << ::std::endl;
//    return false;
//  }
//  if (init_pos_limit) {
//    ::std::cout << RED << "[ERROR moveL] Please wait for the joints to return to position!" << RESET << ::std::endl;
//    return false;
//  }
//
//  if (!cartesian_motion_.moveL(pose, speed, acceleration)) return false;
//
//  // 设置规划结束后的运动状态
//  run_cmd_ = ::ul::std17::RobotCommand::MOVEL;
//  trajectory_planning_length_ = cartesian_motion_.getTrajectoryLength();
//  trajectory_planning_complete_ = true;
//  trajectory_planning_step_ = 0;
//
//  // 阻塞等待
//  this->unblockThread(asynchronous, lock);
//
//  return true;
//}
//
//bool Controller::moveL(const ::std::vector<::std::vector<double>>& path, const double& time, const bool& asynchronous) {
//  // 输入的 path 格式：二维数组，第一个维度表示 path 点数，第二个维度表示左右臂位姿（12维）
//  assert(path.size() >= 2);
//  if (::ul::std17::RobotCommand::TEACH == run_cmd_) {
//    ::std::cout << RED << "[ERROR moveL_Path] Please exit teach mode!" << RESET << ::std::endl;
//    return false;
//  }
//  if (init_pos_limit) {
//    ::std::cout << RED << "[ERROR moveL_Path] Please wait for the joints to return to position!" << RESET << ::std::endl;
//    return false;
//  }
//  if (::ul::std17::RobotCommand::NO_CMD != run_cmd_) {
//    ::std::cout << RED << "[ERROR moveL_Path] Please wait for the robot to stop!" << RESET << ::std::endl;
//    return false;
//  }
//  // this->run_cmd_ = ::ul::std17::RobotCommand::NO_CMD;
//  // 获取上一时刻的双手末端期望位姿
//  ::std::vector<::ul::math::Vector6> cmd_x_last = getForwardKinematics(this->driver_.getLastCommandJointPosition());
//  // 调用接口
//  cartesian_motion_.moveL(cmd_x_last, path, time);
//  ::std::unique_lock<::std::mutex> lock(mtx_);
//
//  // 设置规划结束后的运动状态
//  run_cmd_ = ::ul::std17::RobotCommand::MOVEL_PATH;
//  trajectory_planning_length_ = cartesian_motion_.getTrajectoryLength();
//  trajectory_planning_complete_ = true;
//  trajectory_planning_step_ = 0;
//
//  // 阻塞等待
//  this->unblockThread(asynchronous, lock);
//
//  return true;
//}
//
//bool Controller::moveL(const ::std::vector<::std::vector<double>>& path, const ::std::vector<double>& time, const bool& asynchronous) {
//  // 输入的 path 格式：二维数组，第一个维度表示 path 点数，第二个维度表示左右臂位姿（12维）
//  assert(path.size() >= 2);
//  if (::ul::std17::RobotCommand::TEACH == run_cmd_) {
//    ::std::cout << RED << "[ERROR moveL_Path] Please exit teach mode!" << RESET << ::std::endl;
//    return false;
//  }
//  if (init_pos_limit) {
//    ::std::cout << RED << "[ERROR moveL_Path] Please wait for the joints to return to position!" << RESET << ::std::endl;
//    return false;
//  }
//  if (::ul::std17::RobotCommand::NO_CMD != run_cmd_) {
//    ::std::cout << RED << "[ERROR moveL_Path] Please wait for the robot to stop!" << RESET << ::std::endl;
//    return false;
//  }
//  // this->run_cmd_ = ::ul::std17::RobotCommand::NO_CMD;
//
//  // 获取上一时刻的双手末端期望位姿
//  ::ul::math::Vector modelJointAngles(this->dof_);
//  ::std::vector<::ul::math::Vector6> cmd_x_last = getForwardKinematics(this->driver_.getLastCommandJointPosition());
//  // 调用接口
//  cartesian_motion_.moveL(cmd_x_last, path, time);
//  ::std::unique_lock<::std::mutex> lock(mtx_);
//
//  // 设置规划结束后的运动状态
//  run_cmd_ = ::ul::std17::RobotCommand::MOVEL_PATH;
//  trajectory_planning_length_ = cartesian_motion_.getTrajectoryLength();
//  trajectory_planning_complete_ = true;
//  trajectory_planning_step_ = 0;
//
//  // 阻塞等待
//  this->unblockThread(asynchronous, lock);
//
//  return true;
//}
//
//bool Controller::moveNullSpace(const ::std::vector<::ul::math::Vector6> pose) {
//  ::std::vector<::std::vector<::ul::math::Real>> q_nullspace;
//  q_nullspace = cartesian_motion_.moveNullSpace(pose);
//  moveJ(q_nullspace, q_nullspace.size()*0.1, false);
//  return true;
//}
//
//bool Controller::speedJ(const ::ul::math::Vector& qd, double acceleration, double time) {
//  // ::std::lock_guard<::std::mutex> lock(mtx_);
//
//  if (::ul::std17::RobotCommand::TEACH == run_cmd_) {
//    ::std::cout << RED << "[ERROR speedJ] Please exit teach mode!" << RESET << ::std::endl;
//    return false;
//  }
//  if (init_pos_limit) {
//    ::std::cout << RED << "[ERROR speedJ] Please wait for the joints to return to position!" << RESET << ::std::endl;
//    return false;
//  }
//  if (abs(acceleration) < 1e-8) {
//    ::std::cout << RED << "[ERROR speedJ] Param [acceleration] is zero!" << RESET << ::std::endl;
//    return false;
//  }
//
//  ::std::unique_lock<::std::mutex> lock(mtx_);
//  if (!joints_motion_.speedJ(qd, acceleration, time)) return false;
//
//  // 设置规划结束后的运动状态
//  run_cmd_ = ::ul::std17::RobotCommand::SPEEDJ;
//  // trajectory_planning_length_ = joints_motion_.getTrajectoryLength();
//  trajectory_planning_length_ = time / this->driver_.getUpdateRate();
//  trajectory_planning_complete_ = true;
//  trajectory_planning_step_ = 0;
//
//  // 阻塞等待
//  this->unblockThread(lock);
//  return true;
//}
//
//bool Controller::speedL(const ::std::vector<::ul::math::Vector6> &xd, const double& acceleration, const double& time) {
//  // ::std::unique_lock<::std::mutex> lock(this->mtx_);
//
//  if (::ul::std17::RobotCommand::TEACH == run_cmd_) {
//    ::std::cout << RED << "[ERROR speedL] Please exit teach mode!" << RESET << ::std::endl;
//    return false;
//  }
//  if (init_pos_limit) {
//    ::std::cout << RED << "[ERROR speedL] Please wait for the joints to return to position!" << RESET << ::std::endl;
//    return false;
//  }
//  if (abs(acceleration) < 1e-8) {
//    ::std::cout << RED << "[ERROR speedL] Param [acceleration] is zero!" << RESET << ::std::endl;
//    return false;
//  }
//  ::std::vector<::ul::math::Vector6> init_xd, init_xdd;
//  init_xd = getEEVel(this->driver_.getLastCommandJointPosition(), this->driver_.getLastCommandJointVelocity());
//  init_xdd = getEEAcc(this->driver_.getLastCommandJointPosition(), this->driver_.getLastCommandJointVelocity(), this->driver_.getLastCommandJointAcceleration());
//
//  ::std::unique_lock<::std::mutex> lock(mtx_);
//  if (!cartesian_motion_.speedL(init_xd, init_xdd, xd, acceleration, time)) return false;
//
//  // 设置规划结束后的运动状态
//  run_cmd_ = ::ul::std17::RobotCommand::SPEEDL;
//  // trajectory_planning_length_ = cartesian_motion_.getTrajectoryLength();
//  trajectory_planning_length_ = time / this->driver_.getUpdateRate();
//  trajectory_planning_complete_ = true;
//  trajectory_planning_step_ = 0;
//
//  // 阻塞等待
//  this->unblockThread(lock);
//  return true;
//}
//
//bool Controller::speedStop(::ul::math::Real a) {
//  if (::ul::std17::RobotCommand::TEACH == run_cmd_) {
//    ::std::cout << RED << "[ERROR speedStop] Please exit teach mode!" << RESET << ::std::endl;
//    return false;
//  }
//  if (init_pos_limit) {
//    ::std::cout << RED << "[ERROR speedStop] Please wait for the joints to return to position!" << RESET << ::std::endl;
//    return false;
//  }
//  if (abs(a) < 1e-6) {
//    ::std::cout << RED << "[ERROR speedStop] Param [a] is zero!" << RESET << ::std::endl;
//    return false;
//  }
//
//  // ::std::lock_guard<::std::mutex> lock(mtx_);
//  ::std::unique_lock<::std::mutex> lock(mtx_);
//
//  if (!joints_motion_.speedStop(a)) return false;
//
//  // 设置规划结束后的运动状态
//  this->run_cmd_ = ::ul::std17::RobotCommand::SPEED_STOP;
//  this->trajectory_planning_length_ = joints_motion_.getTrajectoryLength();
//  this->trajectory_planning_complete_ = true;
//  this->trajectory_planning_step_ = 0;
//
//  // 阻塞等待
//  this->unblockThread(false, lock);
//  return true;
//}
//
//bool Controller::loadCSVData(std::vector<std::vector<ul::math::Real>>&& data) {
//  std::cout << YELLOW << "[CSV] Loading file..." << RESET << std::endl;
//  joints_motion_.loadCSVData(std::move(data));
//  std::cout << GREEN << "[CSV] File loaded!" << RESET << std::endl;
//  return true;
//}
//
//bool Controller::CSVTrajectoryMove(bool asynchronous) {
//    if (!joints_motion_.CSVTrajectoryMove()) {
//      return false;
//    }
//    ::std::unique_lock<std::mutex> lock(mtx_);
//    // 设置规划结束后的运动状态
//    run_cmd_ = ::ul::std17::RobotCommand::MOVECSVFILE;
//    trajectory_planning_length_ = joints_motion_.getTrajectoryLength();
//    trajectory_planning_complete_ = true;
//    trajectory_planning_step_ = 0;
//
//    // 阻塞等待
//    this->unblockThread(asynchronous, lock);
//
//  return true;
//}
//
//bool Controller::teachMode(int arm_type) {
//  if (init_pos_limit) {
//    ::std::cout << RED << "[ERROR teachMode] Please wait for the joints to return to position!" << RESET << ::std::endl;
//    return false;
//  }
//
//  if(::ul::std17::RobotCommand::PROTECTED == this->run_cmd_) {
//    ::std::cout << YELLOW << "[WARN] Please deactivate protection mode!" << RESET << std::endl;
//    return false;
//  }
//
//  if (::ul::std17::RobotCommand::TEACH != this->run_cmd_ && ::ul::std17::RobotCommand::NO_CMD != this->run_cmd_) {
//    ::std::cout << RED << "[ERROR teachMode] Never switch to teach mode during robot motion!" << RESET << ::std::endl;
//    return false;
//  }
//
//  bool mod = false;
//  int idx = 0;
//  ::std::vector<uint8_t> mode_operation_temp(dof_, 8);
//  ::std::unique_lock<::std::mutex> lock(this->mtx_);
//  for (int i = 0; i < joints_num_.size(); ++i) {
//    if((arm_type & (1 << i)) && (!(arm_type_teach_ & (1 << i)))) {
//      arm_type_teach_ += (1 << i);
//      mod = true;
//    }
//    if(arm_type_teach_ & (1 << i)) {
//      std::fill(mode_operation_temp.begin() + idx, mode_operation_temp.begin() + idx + joints_num_[i], 10);
//    }
//    idx += joints_num_[i];
//  }
//
//  if (mod && interaction_control_.teachMode(this->run_cmd_, arm_type)) {
//    this->driver_.setJointModeOperation(mode_operation_temp);
//    this->run_cmd_ = ::ul::std17::RobotCommand::TEACH;
//    return true;
//  } else {
//    return false;
//  }
//}
//
//bool Controller::endTeachMode(int arm_type) {
//  if (init_pos_limit) {
//    ::std::cout << RED << "[ERROR endTeachMode] Please wait for the joints to return to position!" << RESET << ::std::endl;
//    return false;
//  }
//  if (this->joints_security_.jointsPosLimitProtection(false) > 0) {
//    ::std::cout << YELLOW <<  "[ERROR endTeachMode] Joint position exceeds the limit!" << RESET << std::endl;
//    return false;
//  }
//
//  bool mod = false;
//  int idx = 0;
//  std::vector<uint8_t> mode_operation_temp = getJointMode();
//  ::std::unique_lock<::std::mutex> lock(this->mtx_);
//  for (int i = 0; i < joints_num_.size(); ++i) {
//    if((arm_type & (1 << i)) && (arm_type_teach_ & (1 << i))) {
//      arm_type_teach_ -= (1 << i);
//      std::fill(mode_operation_temp.begin() + idx, mode_operation_temp.begin() + idx + joints_num_[i], 8);
//      mod = true;
//    }
//    idx += joints_num_[i];
//  }
//
//  if (mod && interaction_control_.endTeachMode(this->run_cmd_, arm_type)) {
//    if (arm_type_teach_ == 0) {
//      mode_operation_temp.assign(dof_, 8);
//      if(this->controller_mode_ != ::ul::controller::ControllerMode::POSITION_CSP) {
//        std::fill_n(mode_operation_temp.begin(), joints_num_[0] + joints_num_[1], 10); // 双臂切换到CST
//      }
//      this->run_cmd_ = ::ul::std17::RobotCommand::NO_CMD;
//    }
//    this->driver_.setJointModeOperation(mode_operation_temp);
//    return true;
//  } else {
//    return false;
//  }
//}
//
//bool Controller::setJointModeOperation(const ::std::vector<uint8_t>& mode_operation) {
//  if (::ul::std17::RobotCommand::TEACH == run_cmd_) {
//    ::std::cout << RED << "[ERROR setJointModeOperation] Please exit teach mode!" << RESET << ::std::endl;
//    return false;
//  }
//  if (init_pos_limit) {
//    ::std::cout << RED << "[ERROR setJointModeOperation] Please wait for the joints to return to position!" << RESET << ::std::endl;
//    return false;
//  }
//  if (mode_operation.size() != this->dof_) return false;
//  ::std::unique_lock<::std::mutex> lock(this->mtx_);
//  // std::vector<uint8_t> mode_operation1(this->dof_, 8);
//  // mode_operation1[8] = 10;
//  // mode_operation1[9] = 10;
//  // mode_operation1[10] = 10;
//  // mode_operation1[11] = 10;
//  // mode_operation1[12] = 10;
//  // mode_operation1[13] = 10;
//  this->driver_.setJointModeOperation(mode_operation);
//  return true;
//}
//
//bool Controller::servoJ(const ::ul::math::Vector& q, const ::ul::math::Real& speed, const ::ul::math::Real& acceleration, const ::ul::math::Real& time, const ::ul::math::Real& lookahead_time, const ::ul::math::Real& gain) {
//  if (lookahead_time < 0.03 || lookahead_time > 0.2) {
//    std::cout << RED << "[Error ServoJ]: lookahead_time is out of the range [0.03, 0.2]!" << RESET << std::endl;
//  return false;
//  }
//
//  if (gain < 100.0 || gain > 1000.0) {
//    std::cout << RED << "[Error ServoJ]: gain is out of the range [100.0, 1000.0]!" << RESET << std::endl;
//    return false;
//  }
//
//  if (::ul::std17::RobotCommand::TEACH == run_cmd_) {
//    ::std::cout << RED << "[ERROR ServoJ] Please exit teach mode!" << RESET << ::std::endl;
//    return false;
//  }
//
//  if (init_pos_limit) {
//    ::std::cout << RED << "[ERROR ServoJ] Please wait for the joints to return to position!" << RESET << ::std::endl;
//    return false;
//  }
//
//  ::std::unique_lock<std::mutex> lock(mtx_);
//  if (::ul::std17::RobotCommand::SERVOJ != this->run_cmd_) {
//    joints_motion_.servoJ_resetParas(lookahead_time, gain);
//  }
//
//  if (!joints_motion_.servoJ(q, speed, acceleration, time)) return false;
//
//  // 设置规划结束后的运动状态
//  run_cmd_ = ::ul::std17::RobotCommand::SERVOJ;
//  trajectory_planning_length_ = joints_motion_.getTrajectoryLength();
//  trajectory_planning_complete_ = true;
//  trajectory_planning_step_ = 0;
//
//  return true;
//}
//
//bool Controller::servoL(const ::std::vector<::ul::math::Vector6>& pose, const ::ul::math::Real& speed, const ::ul::math::Real& acceleration, const ::ul::math::Real& time, const ::ul::math::Real& lookahead_time, const ::ul::math::Real& gain) {
//  if (lookahead_time < 0.03 || lookahead_time > 0.2) {
//    std::cout << RED << "[Error ServoL]: lookahead_time is out of the range [0.03, 0.2]!" << RESET << std::endl;
//    return false;
//  }
//
//  if (gain < 100.0 || gain > 1000.0) {
//    std::cout << RED << "[Error ServoL]: gain is out of the range [100.0, 1000.0]!" << RESET << std::endl;
//    return false;
//  }
//
//  if (::ul::std17::RobotCommand::TEACH == run_cmd_) {
//    ::std::cout << RED << "[ERROR ServoL] Please exit teach mode!" << RESET << ::std::endl;
//    return false;
//  }
//
//  if (init_pos_limit) {
//    ::std::cout << RED << "[ERROR ServoL] Please wait for the joints to return to position!" << RESET << ::std::endl;
//    return false;
//  }
//
//  ::std::vector<::ul::math::Vector6> init_x(2), init_xd(2);
//  if (::ul::std17::RobotCommand::SERVOL != this->run_cmd_) {
//    cartesian_motion_.servoL_resetParas(lookahead_time, gain);
//    init_x = getForwardKinematics(this->driver_.getLastCommandJointPosition());
//    init_xd = getEEVel(this->driver_.getLastCommandJointPosition(), this->driver_.getLastCommandJointVelocity());
//  }
//  ::std::unique_lock<std::mutex> lock(mtx_);
//
//  if (!cartesian_motion_.servoL(init_x, init_xd, pose, speed, acceleration, time)) return false;
//
//  // 设置规划结束后的运动状态
//  run_cmd_ = ::ul::std17::RobotCommand::SERVOL;
//  trajectory_planning_length_ = cartesian_motion_.getTrajectoryLength();
//  trajectory_planning_complete_ = true;
//  trajectory_planning_step_ = 0;
//
//  return true;
//}
//

/*!
 * 线程二：主函数，实时运行
 */
void Controller::run() {

}
//void Controller::run() {
//  ::std::unique_lock<::std::mutex> lock(mtx_);
//#ifdef DEBUG
////  RTIME startTime = rt_timer_read();
////  RTIME testTime = rt_timer_read();
//#endif
//  if (emergency_stop_flag_ && run_cmd_ != ::ul::std17::RobotCommand::PROTECTED) {   // (全身力控保护模式)
//    trajectory_planning_complete_ = false;
//    trajectory_planning_step_ = 0;
//    run_cmd_ = ::ul::std17::RobotCommand::PROTECTED;
//    ::std::cout << YELLOW << "[WARN] Please deactivate protection mode!" << RESET << std::endl;
//    cond_.notify_one();
//  }
//
//  if (first_no_cmd_) {
//    run_cmd_ = ::ul::std17::RobotCommand::NO_CMD; // 防止iddp启动前的调用
//    fsm(run_cmd_, run_mode_, first_no_cmd_, trajectory_planning_length_, trajectory_planning_complete_, trajectory_planning_step_);
//    if (this->joints_security_.jointsPosLimitProtection(true) > 0) {  // 启动前初始位置位于限位之外时的处理
//      init_pos_limit = true;
//      ::std::cout << YELLOW <<  "[WARN] Initial actual joint position exceeds the limit!" << RESET << std::endl;
//      joints_motion_.moveJ(this->driver_.getCommandJointPosition(), 0.5, 0.5);
//
//      // 设置规划结束后的运动状态
//      run_cmd_ = ::ul::std17::RobotCommand::MOVEJ;
//      trajectory_planning_length_ = joints_motion_.getTrajectoryLength();
//      trajectory_planning_complete_ = true;
//      trajectory_planning_step_ = 0;
//    } else {
//      init_pos_limit = false;
//    }
//  } else if(init_pos_limit) {
//    if (run_cmd_ == ::ul::std17::RobotCommand::NO_CMD) {
//      init_pos_limit = false;
//    }
//  }
//  fsm(run_cmd_, run_mode_, first_no_cmd_, trajectory_planning_length_, trajectory_planning_complete_, trajectory_planning_step_);
//
//  // 速度、加速度检测
//  if (!init_pos_limit && (!emergency_stop_flag_) && (run_cmd_ != ::ul::std17::RobotCommand::TEACH)) {  // 回归限位之后，保护生效
//    this->joints_security_.jointsPosLimitProtection(true);
//    if(this->joints_security_.jointsVelAccLimitProtection(run_cmd_) && (!protect_flag_vel_acc_)) { // 需要急停
//      protect_flag_vel_acc_ = true;
//    }
//  }
//
//  interaction_control_.update();
//
//  // 碰撞检测
//  if(collision_detection_on_ && (run_cmd_ != ::ul::std17::RobotCommand::TEACH) && (!emergency_stop_flag_)) {
//    Eigen::Array<bool, Eigen::Dynamic, 1> collision_flag = interaction_control_.getJointCollisionFlag();
//    // 任意一个关节发生碰撞
//    if (collision_flag.any()) {
//      is_collsion_flag_ = true;
//    }
//    if (is_collsion_flag_ && (!emergency_stop_flag_)) {
//      ::std::cout << RED << "collision_flag: " << collision_flag.transpose() << RESET << std::endl;
//      ::std::cout << RED << "tau_est: " << interaction_control_.getTauExt().transpose() << RESET << std::endl;
//    }
//  }
//
//  // 保护制动
//  if((protect_flag_vel_acc_ || is_collsion_flag_) && (!emergency_stop_flag_)) {
//    ///////////////////////////////////双臂力控保护模式/////////////////////////////////////
//    this->interaction_control_.protectedMode(4.0, 0.1);
//    ::std::vector<uint8_t> mode_operation(dof_, 8);
//    ::std::fill_n(mode_operation.begin(), joints_num_[0] + joints_num_[1], 10); // 双臂切换到CST
//    this->driver_.setJointModeOperation(mode_operation);
//    this->interaction_control_.impedanceController_protected();
//    this->run_cmd_ = ::ul::std17::RobotCommand::PROTECTED;
//    emergency_stop_flag_ = true;
//
//    trajectory_planning_complete_ = false;
//    trajectory_planning_step_ = 0;
//    cond_.notify_one();
//    ::std::cout << YELLOW <<  "[WARN] Protection mode has been activated on the robot!" << RESET << std::endl;
//    //////////////////////////////////////////////////////////////////////////
//  }
//
//  if (run_cmd_ != ::ul::std17::RobotCommand::TEACH && (!emergency_stop_flag_)) {
//    if(this->controller_mode_ == ::ul::controller::ControllerMode::POSITION_CST) {
//      interaction_control_.trajectoryTrackingController();
//    } else if (this->controller_mode_ == ::ul::controller::ControllerMode::IMPEDANCE) {
//      interaction_control_.impedanceController();
//    }
//  }
//
//  this->driver_.setLastJointPosition(this->driver_.getCommandJointPosition());
//  this->driver_.setLastJointVelocity(this->driver_.getCommandJointVelocity());
//  this->driver_.setLastJointAcceleration(this->driver_.getCommandJointAcceleration());
//}
//
void Controller::readConfig() {
  ::std::string label;
  joint_names_.clear();

  try {
    ::std::vector<int> alg_nums, driver_nums, urdf_nums;
    ::std::vector<::ul::math::Real> lower_nums, upper_nums;
    ::std::vector<::ul::math::Real> f_a_nums, f_b_nums, f_c_nums;

    // 设置位置差分阈值
    label = "POSITION_THRESHOLD";
    double pos_threshold = this->config_[label].as<double>();
    this->setJointPositionThreshold(pos_threshold);

    // 设置碰撞检测是否开启
    label = "COLLISION_DETECTION_ON";
    this->collision_detection_on_ = this->config_[label].as<bool>();

    // 设置关节位置上下限、关节顺序
    label = "JOINTS_LIST";
    auto joints_list = this->config_[label];
    // 遍历每个关节
    for (const auto& jointNode : joints_list) {
      for (const auto& joint : jointNode) {
        // 获取关节名称
        ::std::string jointName = joint.first.as<::std::string>();
        joint_names_.push_back(jointName);

        // 获取关节属性
        const YAML::Node& jointProps = joint.second;
        // 提取各个数值
        this->offset_vel_on_.push_back(jointProps["offset_vel"].as<int>());
        alg_nums.push_back(jointProps["algNum"].as<int>());
        driver_nums.push_back(jointProps["driverNum"].as<int>());
        f_a_nums.push_back(jointProps["f_a"].as<double>());
        f_b_nums.push_back(jointProps["f_b"].as<double>());
        f_c_nums.push_back(jointProps["f_c"].as<double>());
        ::std::vector<double> numbers = jointProps["pos_limit"].as<std::vector<double>>();
        lower_nums.push_back(numbers[0]);
        upper_nums.push_back(numbers[1]);
      }
    }

    // 设置算法索引、驱动器索引、URDF-Pinocchio索引
    algorithm_idx_ = ::Eigen::Map<::Eigen::VectorXi>(alg_nums.data(), alg_nums.size());

    ::Eigen::VectorXi driver_idx = ::Eigen::Map<::Eigen::VectorXi>(driver_nums.data(), driver_nums.size());
    ::std::vector<std::string> urdf_joint_names = this->getJointName();
    for (const auto& joint_name : urdf_joint_names) {
      auto it = ::std::find(joint_names_.begin(), joint_names_.end(), joint_name);
      if (it != joint_names_.end()) {
        urdf_nums.push_back(::std::distance(joint_names_.begin(), it));
      } else {
        ::std::cout << RED <<  "[Error] config File has no " << joint_name << "." << RESET << std::endl;
        label = "ERROR";  // 触发错误
        auto err = this->config_[label].as<int>();
      }
    }
    ::Eigen::VectorXi urdf_idx = ::Eigen::Map<::Eigen::VectorXi>(urdf_nums.data(), urdf_nums.size());
    this->robot_.setIndexURDF(urdf_idx);
    ::Eigen::VectorXi urdf_idx_inv = this->robot_.getIndexURDFInverse();

    urdf_chain_idx_.resize(2);
    alg_chain_idx_.resize(2);
    std::vector<int> numbers;
    label = "CHAIN_LEFT";
    numbers = config_[label].as<std::vector<int>>();
    this->urdf_chain_idx_[0].resize(numbers.size());
    for (int j = 0; j < numbers.size(); ++j) {
      this->urdf_chain_idx_[0][j] = urdf_idx_inv[numbers[j]];
    }
    this->alg_chain_idx_[0].resize(numbers.size());
    this->alg_chain_idx_[0] = ::Eigen::Map<::Eigen::VectorXi>(numbers.data(), numbers.size());

    numbers.clear();
    label = "CHAIN_RIGHT";
    numbers = config_[label].as<std::vector<int>>();
    this->urdf_chain_idx_[1].resize(numbers.size());
    for (int j = 0; j < numbers.size(); ++j) {
      this->urdf_chain_idx_[1][j] = urdf_idx_inv[numbers[j]];
    }
    this->alg_chain_idx_[1].resize(numbers.size());
    this->alg_chain_idx_[1] = ::Eigen::Map<::Eigen::VectorXi>(numbers.data(), numbers.size());

//    this->cartesian_motion_.setChainIndex(this->urdf_chain_idx_, this->alg_chain_idx_);
    this->interaction_control_.setChainIndex(this->urdf_chain_idx_, this->alg_chain_idx_);

    // 设置关节位置限位
    ::ul::math::Vector lower_limit, upper_limit;
    lower_limit = ::Eigen::Map<::ul::math::Vector>(lower_nums.data(), lower_nums.size());
    upper_limit = ::Eigen::Map<::ul::math::Vector>(upper_nums.data(), upper_nums.size());
    this->setJointPositionLimit(lower_limit, upper_limit);

    // 设置关节摩擦参数
    label = "F_S_K";
    double s_k = this->config_[label].as<double>();
    ::ul::math::Vector f_a, f_b, f_c;
    f_a = ::Eigen::Map<::ul::math::Vector>(f_a_nums.data(), f_a_nums.size());
    f_b = ::Eigen::Map<::ul::math::Vector>(f_b_nums.data(), f_b_nums.size());
    f_c = ::Eigen::Map<::ul::math::Vector>(f_c_nums.data(), f_c_nums.size());
    this->setJointFrictionParas(s_k, f_a, f_b, f_c);

    // 设置控制器参数
    label = "CONTROLLER_PARA_PATH";
    if (!this->interaction_control_.setControllerParas(config_[label].as<std::string>().c_str(), false)) return;

  } catch (const YAML::Exception& e) {
    if (label == "ERROR") {
      ::std::cout << RED <<  "[Error] Joint names in the URDF and config files do not match." << RESET << std::endl;
    } else {
      ::std::cout << RED <<  "[Error init] Config file has no " << label << "!" << RESET << std::endl;
    }
    ::std::exit(1);
  }
}
void Controller::createRobot() {
  robot_cfg_ = ::ul::controller::RobotFactory::createRobot(this->getRobotName());
  robot_cfg_->printInfo();
}

/*!
 * 线程二：Socket服务，用来接收、解析用户指令
 * @param keepRunning: true 表示运行，false 表示停止
 */
void Controller::serverRun(::std::atomic<bool>* keepRunning) {
  try {
    // 设置非阻塞模式示例
    cmd_server_.set_receive_timeout(1000); // 1秒超时
    while (*keepRunning) {
      // 接收数据并传递给socket_cmd_
      if (cmd_server_.receive(socket_cmd_)) {
        // 将接受到的非实时数据写入缓冲区
        double_buffer_.N2RWriteData(socket_cmd_);
      }
      ::std::this_thread::sleep_for(std::chrono::milliseconds(1));  // TODO：后续要改为xenomai的计时
    }
  } catch (const std::exception& e) {
    std::cerr << "CMD server error: " << e.what() << std::endl;
    return;
  }
}

/*!
 * 处理从Socket中接收到的命令socket_cmd_，提取出用户指令参数到缓冲区
 */
void Controller::socketCmdHandler() {
  switch (socket_cmd_.cmd) {
  case ::ul::std17::InterfaceType::MOVEJ:
    // 将MOVEJ的参数提取至缓冲区，需要考虑数组大小
    double_buffer_.N2RWriteData(socket_cmd_);
    socket_cmd_.params["q"].get<::std::vector<double>>();
    break;
  case ::ul::std17::InterfaceType::MOVEL:
    // 将参数提取至缓冲区

    break;
  case ::ul::std17::InterfaceType::UNKNOWN:
    std::cout << "UNKNOWN command received" << std::endl;
    break;
  default:
    std::cerr << "未知命令类型" << std::endl;
    break;
  }
}

//void Controller::cartesianInit(::ul::std17::CartesianConfig& c) {
//  this->cartesian_config_ = c;
//  for (int i = 0; i < 2; ++i) {
//    this->cartesian_config_.joint_idx_first[i] = this->robot_.getJointIndex(c.joint_name_first[i]);
//    this->cartesian_config_.ee_idx[i] = this->robot_.getFrameId(c.ee_name[i]);
//  }
//  this->cartesian_motion_.setCartesianConfig(this->cartesian_config_);
//  this->interaction_control_.setCartesianConfig(this->cartesian_config_);
//  return;
//}
//
//void Controller::setJointsNum(::std::vector<int> joints_num) { this->joints_num_ = joints_num; }
//
//void Controller::setLoadParas(::std::vector<::ul::math::Real>& load_mass, ::std::vector<::ul::math::Vector3>& load_barycenter_trans) {
//  std::lock_guard<std::mutex> lock(mtx_);
//  this->interaction_control_.setLoadParas(load_mass, load_barycenter_trans);
//  return;
//}
//
//void Controller::resetState(std17::RobotCommand& cmd, bool& is_complete, int& step, const ::std::string& task_name) {
//  is_complete = false;
//  step = 0;
//  cmd = ::ul::std17::RobotCommand::NO_CMD;
//  ::std::cout << GREEN << task_name << " complete!" << RESET << ::std::endl;
//}
//
//bool Controller::setJointPosition(const ::ul::math::Vector& q) {
//  // if (run_mode_ == ::ul::std17::RobotRunMode::CSP && run_cmd_ != ::ul::std17::RobotCommand::NO_CMD) {
//  //   ::std::cout << GREEN << "[ERROR setJointPosition] Please wait until the robot stops." << RESET << ::std::endl;
//  //   return false;
//  // }
//  this->driver_.setJointPosition(q);
//  return true;
//}
//
//bool Controller::setJointTorque(const ::ul::math::Vector& tau) {
//  // if (run_cmd_ != ::ul::std17::RobotCommand::NO_CMD) {
//  //   ::std::cout << GREEN << "[ERROR setJointTorque] Please wait until the robot stops." << RESET << ::std::endl;
//  //   return false;
//  // }
//  this->driver_.setJointTorque(tau);
//  return true;
//}
//
//void Controller::sendCurrentJointPosition() {
//  ::ul::math::Vector temp(dof_);
//  temp.setZero();
//  std::vector<uint8_t> mode_operation(dof_, 8);
//  this->driver_.setJointModeOperation(mode_operation);
//  this->driver_.setJointPosition(this->driver_.getJointPosition());
//  this->driver_.setJointVelocity(temp);
//  this->driver_.setJointAcceleration(temp);
//
//  this->driver_.setLastJointPosition(this->driver_.getCommandJointPosition());
//  this->driver_.setLastJointVelocity(temp);
//  this->driver_.setLastJointAcceleration(temp);
//}
//
//void Controller:: unblockThread(const bool& asynchronous, ::std::unique_lock<std::mutex>& lock) {
//  if (!asynchronous) {
//    cond_.wait(lock, [this] {return (run_cmd_ == ::ul::std17::RobotCommand::NO_CMD) || (run_cmd_ == ::ul::std17::RobotCommand::PROTECTED);});
//    cond_.notify_one();
////    ::std::cout << GREEN << "unblock!" << RESET << ::std::endl;
//  }
//}
//
//void Controller::unblockThread(::std::unique_lock<std::mutex>& lock) {
//  cond_.wait(lock, [this] {return trajectory_planning_length_ == trajectory_planning_step_;});
//  cond_.notify_one();
//}
//
//// test
//  std::vector<double> Controller::get_servoJ_x_t() { return this->joints_motion_.get_servoJ_x_t(); };
//  std::vector<double> Controller::get_servoJ_x_t_f() { return this->joints_motion_.get_servoJ_x_t_f(); };

}  // namespace controller
}  // namespace ul
