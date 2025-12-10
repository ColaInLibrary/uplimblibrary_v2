/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-5-21
 * @Version       : 0.0.1
 * @File          : ControllerWrapper.cpp
 ******************************************************************************
 */

#include "ControllerWrapper.h"

ControllerWrapper::ControllerWrapper(const std::string& config_path) : Controller(config_path) {
  // 根据机器人类型创建适配器
  std::string robotName = this->getRobotName();
  if(robotName == "A1_Pro_UpperBody_NW") {
    this->adapter = ::std::make_unique<DualArmAdapter>();
  } else if(robotName == "UpperBody") {
    this->adapter = ::std::make_unique<UpperBodyAdapter>();
  } else if (robotName == "A3_Pro_dual_arm") {
    this->adapter = ::std::make_unique<A3DualArmAdapter>();
  } else if (robotName == "WA1") {
    this->adapter = ::std::make_unique<WheelArmAdapter>();
  } else if (robotName == "WA2") {
    this->adapter = ::std::make_unique<WA2Adapter>();
  } else if (robotName == "WA3") {
    this->adapter = ::std::make_unique<WA3Adapter>();
  } else if (robotName == "Robot_180") {
    this->adapter = ::std::make_unique<Robot180Adapter>();
  } else if (robotName == "I3_dual_arm") {
    this->adapter = ::std::make_unique<I3DualArmAdapter>();
  } else {
    throw std::runtime_error("Invalid robot name. Please check the robot name in urdf.");
  }
  this->adapter->applyCartesianConfig();
  this->cartesianInit(this->adapter->cartesian_config);
  this->setJointsNum(this->adapter->joint_num);
}

ControllerWrapper::~ControllerWrapper() = default;


// 多机型 goHome
bool ControllerWrapper::goHome_adapter(int arm_type) {
  ::ul::math::Vector home;
  adapter->applyHomeInput(home, arm_type);
  if (!this->validateJointInput(home, arm_type)) return false;
  Eigen::VectorXd q_temp = this->getLastCommandJointPosition();
  adapter->applyJointInput(q_temp, home, arm_type);
  return this->moveJ(q_temp, 0.4, 1.4, false);
}

// 多机型 goBack
bool ControllerWrapper::goBack_adapter() {
  return this->goBack(adapter->q_home);
}

// 多机型获取关节实际位置
::ul::math::Vector ControllerWrapper::getJointActualPosition_adapter(int arm_type) {
  return adapter->getJointActualInfo(this->getJointPosition(), arm_type);
}

// 多机型获取关节实际速度
::ul::math::Vector ControllerWrapper::getJointActualVelocity_adapter(int arm_type) {
  return adapter->getJointActualInfo(this->getJointVelocity(), arm_type);
}

// 多机型计算运动学逆解
int ControllerWrapper::getInverseKinematics_adapter(const ::Eigen::VectorXd &pose, ::ul::math::Real q7, ::std::vector<::std::vector<Eigen::VectorXd>>& q, ::std::vector<::std::vector<double>>& phi, int arm_type) {
  q.clear();
  phi.clear();
  if (!this->validateCartesianInput(pose, arm_type)) return false;
  Eigen::VectorXd q_temp = this->getLastCommandJointPosition();
  ::std::vector<double> q7_temp = {q_temp[6], q_temp[13]};
  q7_temp[arm_type - 1] = q7;
  ::std::vector<ul::math::Vector6> pose_temp = this->getForwardKinematics(this->getLastCommandJointPosition());
  adapter->applyCartesianInput(pose_temp, pose, arm_type);
  ::std::vector<::std::vector<Eigen::VectorXd>> q_solutuion_temp;
  ::std::vector<::std::vector<double>> phi_temp;
  int result = this->getInverseKinematics(pose_temp, q7_temp, q_solutuion_temp, phi_temp);
  for (int i = 0; i < 2; ++i) {
    if((arm_type & (1 << i)) && (result & (1 << i))) {
      q.push_back(q_solutuion_temp[arm_type - 1]);
      phi.push_back(phi_temp[arm_type - 1]);
      return arm_type;
    } 
  }
  q.resize(0);
  phi.resize(0);
  return 0;
}

// 多机型双臂奇异状态获取
int ControllerWrapper::isSingular_adapter(const Eigen::VectorXd &q, int arm_type) {
  if (!this->validateJointInput(q, arm_type)) return -1;
  Eigen::VectorXd q_temp = this->getLastCommandJointPosition();
  adapter->applyJointInput(q_temp, q, arm_type);
  return (arm_type & this->isSingular(q_temp));
}

// 多机型设置run_mode
bool ControllerWrapper::setJointModeOperation_adapter(const ::ul::std17::RobotRunMode& run_mode, int arm_type) {
  // if (this->getRobotName() != "UpperBody" || (arm_type != 8)) {
  //   std::cout << RED << "This interface only supports the waist joint of the UpperBody!" << RESET << std::endl;
  //   return false;
  // }
  ::std::vector<uint8_t>mode_operation;
  if (!adapter->applyJointRunMode(run_mode, arm_type, mode_operation)) return false;
  Eigen::Map<const Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>> mode_operation_seg(mode_operation.data(), mode_operation.size());

  Eigen::Matrix<uint8_t, Eigen::Dynamic, 1> mode_operation_temp_vec = Eigen::Map<Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>>(this->getJointMode().data(), this->getDof());
  adapter->applyJointInput(mode_operation_temp_vec, mode_operation_seg, arm_type);
  
  ::std::vector<uint8_t> mode_operation_temp = std::vector<uint8_t>{mode_operation_temp_vec.begin(), mode_operation_temp_vec.end()};
  return this->setJointModeOperation(mode_operation_temp);
}

// 多机型设置controller_mode
bool ControllerWrapper::setControllerMode_adapter(::ul::controller::ControllerMode &mode) {
  return this->setControllerMode(mode);
}

// 多机型设置控制参数
bool ControllerWrapper::setControllerPara_adapter(::ul::controller::ParasList &para_label, const Eigen::VectorXd &para, int arm_type) {
  if (!this->validateJointInput(para, arm_type)) return false;
  Eigen::VectorXd para_temp;
  this->getControllerPara(para_label, para_temp);
  adapter->applyJointInput(para_temp, para, arm_type);
  return this->setControllerPara(para_label, para_temp);
}

// 多机型获取控制参数
::ul::math::Vector ControllerWrapper::getControllerPara_adapter(::ul::controller::ParasList &para_label, int arm_type) {
  Eigen::VectorXd para_temp;
  this->getControllerPara(para_label, para_temp);
  return adapter->getJointActualInfo(para_temp, arm_type);
}

// 多机型设置关节位置
bool ControllerWrapper::setJointPosition_adapter(const Eigen::VectorXd &q, int arm_type) {
  if (!this->validateJointInput(q, arm_type)) return false;
  Eigen::VectorXd q_temp = this->getLastCommandJointPosition();
  adapter->applyJointInput(q_temp, q, arm_type);
  return this->setJointPosition(q_temp);
}

// 多机型设置关节力矩
bool ControllerWrapper::setJointTorque_adapter(const Eigen::VectorXd &tau, int arm_type) {
  if (!this->validateJointInput(tau, arm_type)) return false;
  Eigen::VectorXd tau_temp = this->getJointTorque();
  adapter->applyJointInput(tau_temp, tau, arm_type);
  return this->setJointTorque(tau_temp);
}

// 多机型 teachMode
bool ControllerWrapper::teachMode_adapter(int arm_type) {
  if (this->getRobotName() == "WA1" && (arm_type & 8)) {
    std::cout << RED << "The waist_lift does not support teach mode!" << RESET << std::endl;
    return false;
  }
  // ::std::vector<uint8_t>mode_operation;
  // if (!adapter->applyJointRunMode(::ul::std17::RobotRunMode::CST, arm_type, mode_operation)) return false;
  // Eigen::Map<const Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>> mode_operation_seg(mode_operation.data(), mode_operation.size());

  // // Eigen::Matrix<uint8_t, Eigen::Dynamic, 1> mode_operation_temp_vec = Eigen::Map<Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>>(this->getJointMode().data(), this->getDof());
  // Eigen::Matrix<uint8_t, Eigen::Dynamic, 1> mode_operation_temp_vec = Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>::Constant(this->getDof(), 8);
  // adapter->applyJointInput(mode_operation_temp_vec, mode_operation_seg, arm_type);
  
  // ::std::vector<uint8_t> mode_operation_temp = std::vector<uint8_t>{mode_operation_temp_vec.begin(), mode_operation_temp_vec.end()};
  // if (this->setJointModeOperation(mode_operation_temp)) {
  //   return this->teachMode();
  // } else {
  //   return false;
  // }
  return this->teachMode(arm_type);
}

// 多机型 endTeachMode
bool ControllerWrapper::endTeachMode_adapter(int arm_type) {
  if (this->getRobotName() == "WA1" && (arm_type & 8)) {
    std::cout << RED << "The waist_lift does not support teach mode!" << RESET << std::endl;
    return false;
  }
  // ::std::vector<uint8_t>mode_operation;
  // if (!adapter->applyJointRunMode(::ul::std17::RobotRunMode::CSP, arm_type, mode_operation)) return false;
  // Eigen::Map<const Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>> mode_operation_seg(mode_operation.data(), mode_operation.size());

  // Eigen::Matrix<uint8_t, Eigen::Dynamic, 1> mode_operation_temp_vec = Eigen::Map<Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>>(this->getJointMode().data(), this->getDof());
  // adapter->applyJointInput(mode_operation_temp_vec, mode_operation_seg, arm_type);
  
  // ::std::vector<uint8_t> mode_operation_temp = std::vector<uint8_t>{mode_operation_temp_vec.begin(), mode_operation_temp_vec.end()};
  
  // if (!this->endTeachMode()) return false;
  // return this->setJointModeOperation(mode_operation_temp);
  return this->endTeachMode(arm_type);
}

// 多机型 loadCSVData
bool ControllerWrapper::loadCSVData_adapter(const ::std::vector<::std::vector<::ul::math::Real>>&& data, int arm_type) {
  if (!this->validateJointInput(data[0], arm_type)) return false;

  ::std::vector<::std::vector<::ul::math::Real>> data_all;
  data_all.reserve(data.size());
  const Eigen::VectorXd& q_temp = this->getLastCommandJointPosition();

  adapter->applyJointPathInput(data_all, data, q_temp, arm_type);
  return this->loadCSVData(::std::move(data_all)); 
}

// 多机型 moveJ
bool ControllerWrapper::moveJ_adapter(const Eigen::VectorXd& q, ::ul::math::Real speed, ::ul::math::Real acceleration, bool asynchronous, int arm_type) {
  if (!this->validateJointInput(q, arm_type)) return false;
  Eigen::VectorXd q_temp = this->getLastCommandJointPosition();
  adapter->applyJointInput(q_temp, q, arm_type);
  return this->moveJ(q_temp, speed, acceleration, asynchronous);
}

// 多机型 moveJ path，最终 time
bool ControllerWrapper::moveJ_adapter(const std::vector<std::vector<::ul::math::Real>>&& path, const ::ul::math::Real& time, bool asynchronous, int arm_type) {
  if (!this->validateJointInput(path[0], arm_type)) return false;

  ::std::vector<::std::vector<::ul::math::Real>> path_all;
  path_all.reserve(path.size());
  const Eigen::VectorXd& q_temp = this->getLastCommandJointPosition();

  adapter->applyJointPathInput(path_all, path, q_temp, arm_type);
  return this->moveJ(path_all, time, asynchronous);
}

// 多机型 moveJ path，路径 time
bool ControllerWrapper::moveJ_adapter(const std::vector<std::vector<::ul::math::Real>>&& path, const std::vector<::ul::math::Real>& time, bool asynchronous, int arm_type) {
  if (path.size() != time.size()) return false;
  if (!this->validateJointInput(path[0], arm_type)) return false;

  ::std::vector<::std::vector<::ul::math::Real>> path_all;
  path_all.reserve(path.size());
  const Eigen::VectorXd& q_temp = this->getLastCommandJointPosition();
  
  adapter->applyJointPathInput(path_all, path, q_temp, arm_type);
  return this->moveJ(path_all, time, asynchronous);
}

// 多机型 moveJ_spline
bool ControllerWrapper::moveJ_adapter(const Eigen::VectorXd &q, const Eigen::VectorXd &qd, ::ul::math::Real freq, bool asynchronous, int arm_type) {
  Eigen::VectorXd q_copy = q;
  Eigen::VectorXd qd_copy = qd;
  if(this->getRobotName() == "I3_dual_arm" || this->getRobotName() == "Robot_180") {
    if(arm_type == 1 || arm_type == 2) {
      if ((q.size() == 7) && (qd.size() == 7)) {
        int size = adapter->joint_num[arm_type - 1];
        q_copy.resize(size);
        qd_copy.resize(size);
        q_copy = q.head(size);
        qd_copy = qd.head(size);
      }
    } else if(arm_type == 3) {
      if ((q.size() == 14) && (qd.size() == 14)) {
        int size = adapter->joint_num[0] + adapter->joint_num[1];
        q_copy.resize(size);
        qd_copy.resize(size);
        q_copy << q.head(adapter->joint_num[0]), q.segment(7, adapter->joint_num[1]);
        qd_copy << qd.head(adapter->joint_num[0]), qd.segment(7, adapter->joint_num[1]);
      }
    }
  }
  if ((!this->validateJointInput(q_copy, arm_type)) || (!this->validateJointInput(qd_copy, arm_type))) return false;
  Eigen::VectorXd q_temp = this->getLastCommandJointPosition();
  adapter->applyJointInput(q_temp, q_copy, arm_type);
  Eigen::VectorXd qd_temp = this->getLastCommandJointVelocity();
  adapter->applyJointInput(qd_temp, qd_copy, arm_type);
  return this->moveJ(q_temp, qd_temp, freq, asynchronous);
}

// moveJ_pose
bool ControllerWrapper::moveJ_adapter(const ::Eigen::VectorXd &pose, ::ul::math::Real q7, ::ul::math::Real speed, ::ul::math::Real acceleration, bool asynchronous, int arm_type) {
  if (!this->validateCartesianInput(pose, arm_type)) return false;
  Eigen::VectorXd q_temp = this->getLastCommandJointPosition();
  ::std::vector<double> q7_temp = {q_temp[6], q_temp[13]};
  q7_temp[arm_type - 1] = q7;
  ::std::vector<ul::math::Vector6> pose_temp = this->getForwardKinematics(this->getLastCommandJointPosition());
  adapter->applyCartesianInput(pose_temp, pose, arm_type);
  return this->moveJ(pose_temp, q7_temp, speed, acceleration, asynchronous);
}

// moveL
bool ControllerWrapper::moveL_adapter(const ::Eigen::VectorXd &pose, ::ul::math::Real speed, ::ul::math::Real acceleration, bool asynchronous, int arm_type) {
  if (!this->validateCartesianInput(pose, arm_type)) return false;
  ::std::vector<ul::math::Vector6> pose_temp = this->getForwardKinematics(this->getLastCommandJointPosition());
  adapter->applyCartesianInput(pose_temp, pose, arm_type);
  return this->moveL(pose_temp, speed, acceleration, asynchronous);
}

// moveL path，最终 time
bool ControllerWrapper::moveL_adapter(const std::vector<std::vector<::ul::math::Real>> &path, const double &time, const bool& asynchronous, int arm_type) {
  if (!this->validateCartesianInput(path[0], arm_type)) return false;
  ::std::vector<::std::vector<::ul::math::Real>> target;
  ::std::vector<ul::math::Vector6> pose_temp = this->getForwardKinematics(this->getLastCommandJointPosition());
  adapter->applyCartesianPathInput(target, path, pose_temp, arm_type);
  return this->moveL(target, time, asynchronous);
}

// moveL path，路径 time
bool ControllerWrapper::moveL_adapter(const std::vector<std::vector<::ul::math::Real>> &path, const std::vector<::ul::math::Real>& time, const bool& asynchronous, int arm_type) {
  if (!this->validateCartesianInput(path[0], arm_type)) return false;
  ::std::vector<::std::vector<::ul::math::Real>> target;
  ::std::vector<ul::math::Vector6> pose_temp = this->getForwardKinematics(this->getLastCommandJointPosition());
  adapter->applyCartesianPathInput(target, path, pose_temp, arm_type);
  return this->moveL(target, time, asynchronous);
}

// 多机型 servoJ
bool ControllerWrapper::servoJ_adapter(const Eigen::VectorXd &q, double speed, double acceleration, double time, double lookahead_time, double gain, int arm_type) {
  if (!this->validateJointInput(q, arm_type)) return false;
  Eigen::VectorXd q_temp = this->getLastCommandJointPosition();
  adapter->applyJointInput(q_temp, q, arm_type);
  return this->servoJ(q_temp, speed, acceleration, time, lookahead_time, gain);
}

// 多机型 servoL
bool ControllerWrapper::servoL_adapter(const ::Eigen::VectorXd &pose, double speed, double acceleration, double time, double lookahead_time, double gain, int arm_type) {
  if (!this->validateCartesianInput(pose, arm_type)) return false;
  ::std::vector<ul::math::Vector6> pose_temp = this->getForwardKinematics(this->getLastCommandJointPosition());
  adapter->applyCartesianInput(pose_temp, pose, arm_type);
  return this-> servoL(pose_temp, speed, acceleration, time, lookahead_time, gain);
}

// 多机型 speedJ
bool ControllerWrapper::speedJ_adapter(const Eigen::VectorXd &qd, double acceleration, double time, int arm_type) {
  if (!this->validateJointInput(qd, arm_type)) return false;
  Eigen::VectorXd qd_temp = this->getLastCommandJointVelocity();
  adapter->applyJointInput(qd_temp, qd, arm_type);
  return this->speedJ(qd_temp, acceleration, time);
}

// 多机型 speedL
bool ControllerWrapper::speedL_adapter(const ::Eigen::VectorXd &xd, const double& acceleration, const double& time, int arm_type) {
  if (!this->validateCartesianInput(xd, arm_type)) return false;
  ::std::vector<ul::math::Vector6> xd_temp = this->getEEVel(this->getLastCommandJointPosition(), this->getLastCommandJointVelocity());
  adapter->applyCartesianInput(xd_temp, xd, arm_type);
  return this->speedL(xd_temp, acceleration, time);
}