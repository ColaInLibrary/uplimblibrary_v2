#include "UplimbController.h"
#include <ul/controller/Controller.h>

#include <memory>
// #ifdef REAL_MODE
// #include <timer.h>
// #endif

// 智能指针懒初始化，这时还没有实例化
::std::unique_ptr<::ul::controller::Controller> controller_ptr;

// 初始化函数
void init(std::string config_path) {
  ::std::cout << "[INFO] uplimb_library Initializing... " << ::std::endl;
  std::string yaml_path;
   const char* path_env = std::getenv("UPLIMB_CONFIG_FILE_PATH");
  if (config_path.empty()) {
    yaml_path = path_env ? path_env : "";
  } else {
    yaml_path = config_path;
  }
  if (yaml_path.empty()) {
    throw std::runtime_error("\n[ERROR] uplimb_library initialization failed.\n"
        "Please call 'init()' using one of the following two methods:\n"
        " - init(); // CONFIG_FILE_PATH is defined.\n"
        " - init(your path/robot_define.yaml); // Defiend the config file path.");
  }
  if (!controller_ptr) {
    controller_ptr = ::std::make_unique<::ul::controller::Controller>(yaml_path);
    ::std::cout << GREEN << "[INFO] config_path: " << yaml_path << RESET << std::endl;
    ::std::cout << GREEN << "[INFO] uplimb_library initialized successfully! " << RESET << std::endl;
  }
}

::ul::controller::Controller& getController() {
  if (!controller_ptr) {
    throw std::runtime_error("[ERROR] uplimb_library not initialized. Call 'init()' first.");
  }
  return *controller_ptr;
}

//ControllerWrapper& getControllerWrapper() {
//  if (!controller_wrapper_ptr) {
//    throw std::runtime_error("[ERROR] uplimb_library not initialized. Call 'init()' first.");
//  }
//  return *controller_wrapper_ptr;
//}
//
//int getInverseKinematics(const CartersianPose& pose,
//                          const q7_Type& q7,
//                          ::std::vector<::std::vector<Eigen::VectorXd>>& q,
//                          ::std::vector<::std::vector<double>>& phi,
//                          int arm_type) {
//  auto& controller_wrapper = getControllerWrapper();
//  int result = 0;
//  std::visit([&](auto&& pose_value) {
//    using PoseT = std::decay_t<decltype(pose_value)>;
//    std::visit([&](auto&& q7_value) {
//      using Q7T = std::decay_t<decltype(q7_value)>;
//      if constexpr (std::is_same_v<PoseT, Eigen::Matrix<double, 6, 1>> && std::is_same_v<Q7T, double>) {
//        result = controller_wrapper.getInverseKinematics_adapter(pose_value, q7_value, q, phi, arm_type);
//      } else if constexpr (std::is_same_v<PoseT, std::vector<Eigen::Matrix<double, 6, 1>>> && std::is_same_v<Q7T, std::vector<double>>) {
//        if (arm_type == 3) {
//          result = controller_wrapper.getInverseKinematics(pose_value, q7_value, q, phi);
//        } else if (arm_type == 1 || arm_type == 2) {
//          result = controller_wrapper.getInverseKinematics_adapter(pose_value[arm_type - 1], q7_value[arm_type - 1], q, phi, arm_type);
//        } else {
//          std::cerr << RED << "[Error] Mismatched types between xd and arm_type!" << RESET << std::endl;
//        }
//      } else {
//        std::cerr << "[Error] Mismatched types between pose and q7!" << std::endl;
//      }
//    }, q7);
//  }, pose);
//  return result;
//}
//
//bool moveJ(const Eigen::VectorXd &q, double speed, double acceleration, bool asynchronous, int arm_type) {
//  auto &controller_wrapper = getControllerWrapper();
//  if (!controller_wrapper.moveJ_adapter(q, speed, acceleration, asynchronous, arm_type)) return false;
//  return true;
//}
//
//bool moveJ(const CartersianPose& pose, const q7_Type& q7, double speed, double acceleration, bool asynchronous, int arm_type) {
//  auto& controller_wrapper = getControllerWrapper();
//  bool result = false;
//  std::visit([&](auto&& pose_value) {
//    using PoseT = std::decay_t<decltype(pose_value)>;
//    std::visit([&](auto&& q7_value) {
//      using Q7T = std::decay_t<decltype(q7_value)>;
//      if constexpr (std::is_same_v<PoseT, Eigen::Matrix<double, 6, 1>> && std::is_same_v<Q7T, double>) {
//        result = controller_wrapper.moveJ_adapter(pose_value, q7_value, speed, acceleration, asynchronous, arm_type);
//      } else if constexpr (std::is_same_v<PoseT, std::vector<Eigen::Matrix<double, 6, 1>>> && std::is_same_v<Q7T, std::vector<double>>) {
//        if (arm_type == 3) {
//          result = controller_wrapper.moveJ(pose_value, q7_value, speed, acceleration, asynchronous);
//        } else if (arm_type == 1 || arm_type == 2) {
//          result = controller_wrapper.moveJ_adapter(pose_value[arm_type - 1], q7_value[arm_type - 1], speed, acceleration, asynchronous, arm_type);
//        } else {
//          std::cerr << RED << "[Error] Mismatched types between xd and arm_type!" << RESET << std::endl;
//        }
//      } else {
//        std::cerr << "[Error] Mismatched types between pose and q7!" << std::endl;
//      }
//    }, q7);
//  }, pose);
//  return result;
//}
//
//bool moveJ(const std::vector<std::vector<double>> &path, const double& time, bool asynchronous, int arm_type) {
//  auto& controller_wrapper = getControllerWrapper();
//  if (!controller_wrapper.moveJ_adapter(::std::move(path), time, asynchronous, arm_type)) return false;
//  return true;
//}
//
//bool moveJ(const std::vector<std::vector<double>> &path, const std::vector<double>& time, bool asynchronous, int arm_type) {
//  auto& controller_wrapper = getControllerWrapper();
//  if (!controller_wrapper.moveJ_adapter(::std::move(path), time, asynchronous, arm_type)) return false;
//  return true;
//}
//
//bool moveJ(const Eigen::VectorXd &q, double freq, bool asynchronous, int arm_type) {
//  auto &controller_wrapper = getControllerWrapper();
//  Eigen::VectorXd qd_zero = Eigen::VectorXd::Zero(q.size());
//  if (!controller_wrapper.moveJ_adapter(q, qd_zero, freq, asynchronous, arm_type)) return false;
//  return true;
//}
//
//bool moveJ(const Eigen::VectorXd &q, const Eigen::VectorXd &qd, double freq, bool asynchronous, int arm_type) {
//  auto &controller_wrapper = getControllerWrapper();
//  if (!controller_wrapper.moveJ_adapter(q, qd, freq, asynchronous, arm_type)) return false;
//  return true;
//}
//
//bool moveL(const ::std::vector<::Eigen::Matrix<double, 6, 1>> &pose, const ::Eigen::VectorXd& ref_q, double speed, double acceleration, bool asynchronous) {
//  auto& controller = getController();
//  if (!controller.moveL(pose, ref_q, speed, acceleration, asynchronous)) return false;
//  return true;
//}
//
//bool moveL(const CartersianPose& pose, double speed, double acceleration, bool asynchronous, int arm_type) {
//  // if (arm_type != 1 && arm_type != 2) {
//  //   std::cerr << RED << "[Error moveL] The current version only supports single-arm use!" << RESET << std::endl;
//  //   return false;
//  // }
//  auto& controller_wrapper = getControllerWrapper();
//  bool result = false;
//  std::visit([&](auto&& value) {
//    using T = std::decay_t<decltype(value)>;
//    if constexpr (std::is_same_v<T, Eigen::Matrix<double, 6, 1>>) {
//      result = controller_wrapper.moveL_adapter(value, speed, acceleration, asynchronous, arm_type);
//    } else if constexpr (std::is_same_v<T, std::vector<Eigen::Matrix<double, 6, 1>>>) {
//      if (arm_type == 3) {
//        result = controller_wrapper.moveL(value, speed, acceleration, asynchronous);
//      } else if (arm_type == 1 || arm_type == 2) {
//        result = controller_wrapper.moveL_adapter(value[arm_type -1], speed, acceleration, asynchronous, arm_type);
//      } else {
//        std::cerr << RED << "[Error] Mismatched types between xd and arm_type!" << RESET << std::endl;
//      }
//    } else {
//      static_assert(always_false<T>::value, "[Error] Invalid CartersianPose input!");
//    }
//  }, pose);
//  return result;
//}
//
//bool moveL(const std::vector<std::vector<double>> &path, const double &time, const bool& asynchronous, int arm_type) {
//  auto& controller_wrapper = getControllerWrapper();
//  if (!controller_wrapper.moveL_adapter(path, time, asynchronous, arm_type)) return false;
//  return true;
//}
//
//bool moveL(const std::vector<std::vector<double>> &path, const ::std::vector<double> &time, const bool& asynchronous, int arm_type) {
//  auto& controller_wrapper = getControllerWrapper();
//  if (!controller_wrapper.moveL_adapter(path, time, asynchronous, arm_type)) return false;
//  return true;
//}
//
//bool moveNullSpace(const ::std::vector<::ul::math::Vector6>& pose) {
//  auto& controller = getController();
//  if (!controller.moveNullSpace(pose)) return false;
//  return true;
//}
//
//bool speedJ(const Eigen::VectorXd &qd, double acceleration, double time, int arm_type) {
//  auto& controller_wrapper = getControllerWrapper();
//  if (!controller_wrapper.speedJ_adapter(qd, acceleration, time, arm_type)) return false;
//  return true;
//}
//
//bool speedL(const CartersianVel& xd, const double& acceleration, const double& time, int arm_type) {
//  auto& controller_wrapper = getControllerWrapper();
//  bool result = false;
//  std::visit([&](auto&& value) {
//    using T = std::decay_t<decltype(value)>;
//    if constexpr (std::is_same_v<T, Eigen::Matrix<double, 6, 1>>) {
//      result = controller_wrapper.speedL_adapter(value, acceleration, time, arm_type);
//    } else if constexpr (std::is_same_v<T, std::vector<Eigen::Matrix<double, 6, 1>>>) {
//      if (arm_type == 3) {
//        result = controller_wrapper.speedL(value, acceleration, time);
//      } else if (arm_type == 1 || arm_type == 2) {
//        result = controller_wrapper.speedL_adapter(value[arm_type - 1], acceleration, time, arm_type);
//      } else {
//        std::cerr << RED << "[Error] Mismatched types between xd and arm_type!" << RESET << std::endl;
//      }
//    } else {
//      static_assert(always_false<T>::value, "[Error] Invalid CartersianVel input!");
//    }
//  }, xd);
//  return result;
//}
//
//bool speedStop(double a) {
//  auto& controller = getController();
//  if (!controller.speedStop(a)) return false;
//  return true;
//}
//
//bool servoJ(const Eigen::VectorXd &q, double speed, double acceleration, double time, double lookahead_time, double gain, int arm_type) {
//  auto &controller_wrapper = getControllerWrapper();
//  if (!controller_wrapper.servoJ_adapter(q, speed, acceleration, time, lookahead_time, gain, arm_type)) return false;
//  return true;
//}
//
//bool servoL(const CartersianPose& pose, double speed, double acceleration, double time, double lookahead_time, double gain, int arm_type) {
//  auto& controller_wrapper = getControllerWrapper();
//  bool result = false;
//  std::visit([&](auto&& value) {
//    using T = std::decay_t<decltype(value)>;
//    if constexpr (std::is_same_v<T, Eigen::Matrix<double, 6, 1>>) {
//      result = controller_wrapper.servoL_adapter(value, speed, acceleration, time, lookahead_time, gain, arm_type);
//    } else if constexpr (std::is_same_v<T, std::vector<Eigen::Matrix<double, 6, 1>>>) {
//      if (arm_type == 3) {
//        result = controller_wrapper.servoL(value, speed, acceleration, time, lookahead_time, gain);
//      } else if (arm_type == 1 || arm_type == 2) {
//        result = controller_wrapper.servoL_adapter(value[arm_type - 1], speed, acceleration, time, lookahead_time, gain, arm_type);
//      } else {
//        std::cerr << RED << "[Error] Mismatched types between xd and arm_type!" << RESET << std::endl;
//      }
//    } else {
//      static_assert(always_false<T>::value, "[Error] Invalid CartersianPose input!");
//    }
//  }, pose);
//  return result;
//}
//
////bool servoStop(double a, int arm_type) {
////    if (arm_type == 0) {
////        if (!controller.servoStop(a))
////        return false;
////    } else if (arm_type == 1) {
////        if (!controller_right.servoStop(a))
////        return false;
////    }
////    return true;
////}
//
//bool goHome(int arm_type) {
//  auto &controller_wrapper = getControllerWrapper();
//  if (!controller_wrapper.goHome_adapter(arm_type)) return false;
//  return true;
//}
//
//bool goBack() {
//  auto &controller_wrapper = getControllerWrapper();
//  return controller_wrapper.goBack_adapter();
//}
//
//::Eigen::VectorXd getJointActualPositions(int arm_type) {
//  auto& controller_wrapper = getControllerWrapper();
//  return controller_wrapper.getJointActualPosition_adapter(arm_type);
//}
//
//
//::Eigen::VectorXd getJointActualVelocities(int arm_type) {
//  auto& controller_wrapper = getControllerWrapper();
//  return controller_wrapper.getJointActualVelocity_adapter(arm_type);
//}
//
//::std::vector<ul::math::Vector6> getForwardKinematics() {
//  auto& controller = getController();
//  return controller.getForwardKinematics();
//}
//
//Eigen::VectorXd getForwardKinematics(int arm_type) {
//  auto& controller = getController();
//  if (arm_type != 1 && arm_type != 2) {
//    ::std::cout << RED << "[Error getForwardKinematics] arm_type is wrong!" << RESET << std::endl;
//    return Eigen::VectorXd::Zero(6);
//  }
//  return controller.getForwardKinematics()[arm_type - 1];
//}
//
//std::vector<::Eigen::Matrix<double, 6, 1>> getForwardKinematics(const Eigen::VectorXd &q) {
//  auto& controller = getController();
//  return controller.getForwardKinematics(q);
//}
//
//Eigen::VectorXd getForwardKinematics(const Eigen::VectorXd &q, int arm_type) {
//  auto& controller = getController();
//  if (arm_type != 1 && arm_type != 2) {
//    ::std::cout << RED << "[Error getForwardKinematics] arm_type is wrong!" << RESET << std::endl;
//    return Eigen::VectorXd::Zero(6);
//  }
//  return controller.getForwardKinematics(q)[arm_type - 1];
//}
//
//::std::vector<::ul::math::Vector6> getEEVel() {
//  auto& controller = getController();
//  return controller.getEEVel();
//}
//
//Eigen::VectorXd getEEVel(int arm_type) {
//  auto& controller = getController();
//  if (arm_type != 1 && arm_type != 2) {
//    ::std::cout << RED << "[Error getEEVel] arm_type is wrong!" << RESET << std::endl;
//    return Eigen::VectorXd::Zero(6);
//  }
//  return controller.getEEVel()[arm_type - 1];
//}
//
//bool getGravityTorque(Eigen::VectorXd &tau_g) {
//  auto& controller = getController();
//  tau_g = controller.getGravityTorque();
//  return true;
//}
//
//::std::vector<Eigen::MatrixXd> getJacobian() {
//  auto& controller = getController();
//  return controller.getJacobian();
//}
//
//Eigen::MatrixXd getJacobian(int arm_type) {
//  auto& controller = getController();
//  if (arm_type != 1 && arm_type != 2) {
//    ::std::cout << RED << "[Error getJacobian] arm_type is wrong!" << RESET << std::endl;
//    return Eigen::MatrixXd::Zero(6, controller.getDof());
//  }
//  return controller.getJacobian()[arm_type - 1];
//}
//
//bool getCSVFile(const std::string& file_name, int arm_type) {
//  ::std::vector<::std::vector<::ul::math::Real>> data;
//  if (!::ul::std17::getCSVData(file_name, data)) {
//    return false;
//  }
//
//  auto& controller_wrapper = getControllerWrapper();
//  return controller_wrapper.loadCSVData_adapter(::std::move(data), arm_type);
//  return true;
//}
//
//bool CSVFileMove(bool asynchronous) {
//  auto& controller = getController();
//  return controller.CSVTrajectoryMove(asynchronous);
//}
//
//bool teachMode(int arm_type) {
//  // auto& controller = getController();
//  // return controller.teachMode();
//  auto& controller_wrapper = getControllerWrapper();
//  return controller_wrapper.teachMode_adapter(arm_type);
//}
//
//bool endTeachMode(int arm_type) {
//  // auto& controller = getController();
//  // return controller.endTeachMode();
//  auto& controller_wrapper = getControllerWrapper();
//  return controller_wrapper.endTeachMode_adapter(arm_type);
//}
//
::std::size_t getDof() {
  auto& controller = getController();
  return controller.getDof();
}
//
void getDriverInfo(UPLIMB_INPUT_INFO_STRUCT *recv_info) {
  auto& controller = getController();
  static ::ul::math::Vector q(controller.getDof());
  static ::ul::math::Vector qd(controller.getDof());
  static ::ul::math::Vector tau(controller.getDof());
  static ::std::vector<::std::uint16_t> statuswd(controller.getDof());
  static ::std::vector<::std::uint8_t> mode(controller.getDof());
  unsigned long long int upTime;

  for (int j = 0; j < controller.getDof(); ++j) {
    mode[j] = recv_info->mode[j];
    q[j] = recv_info->act_pos[j];
    qd[j] = recv_info->act_vel[j];
    statuswd[j] = recv_info->statuswd[j];
    tau[j] = recv_info->act_tau[j];
  }
  //////////////////////R180///////////////////////////////////
  // for (int j = 0; j < 10; ++j) {
  //   mode[j] = recv_info->mode[j];
  //   q[j] = recv_info->act_pos[j];
  //   qd[j] = recv_info->act_vel[j];
  //   statuswd[j] = recv_info->statuswd[j];
  //   tau[j] = recv_info->act_tau[j];
  // }
  // for (int j = 14; j < 18; ++j) {
  //   mode[j-4] = recv_info->mode[j];
  //   q[j-4] = recv_info->act_pos[j];
  //   qd[j-4] = recv_info->act_vel[j];
  //   statuswd[j-4] = recv_info->statuswd[j];
  //   tau[j-4] = recv_info->act_tau[j];
  // }
  //////////////////////R180///////////////////////////////////
  //////////////////////I3///////////////////////////////////
  // for (int j = 0; j < 4; ++j) {
  //   mode[j] = recv_info->mode[j];
  //   q[j] = recv_info->act_pos[j];
  //   qd[j] = recv_info->act_vel[j];
  //   statuswd[j] = recv_info->statuswd[j];
  //   tau[j] = recv_info->act_tau[j];
  // }
  // for (int j = 7; j < 11; ++j) {
  //   mode[j-3] = recv_info->mode[j];
  //   q[j-3] = recv_info->act_pos[j];
  //   qd[j-3] = recv_info->act_vel[j];
  //   statuswd[j-3] = recv_info->statuswd[j];
  //   tau[j-3] = recv_info->act_tau[j];
  // }
  // for (int j = 14; j < 16; ++j) {
  //   mode[j-6] = recv_info->mode[j];
  //   q[j-6] = recv_info->act_pos[j];
  //   qd[j-6] = recv_info->act_vel[j];
  //   statuswd[j-6] = recv_info->statuswd[j];
  //   tau[j-6] = recv_info->act_tau[j];
  // }
  //////////////////////I3///////////////////////////////////
//  upTime = recv_info->upTime;
//
  ::ul::math::Real up_time_millisecond = upTime / 1000000.0;
  controller.getJointMode(mode);
  controller.getJointPosition(q);
  controller.getJointVelocity(qd);
  controller.getJointStatusWord(statuswd);
  controller.getJointTorque(tau);
  controller.getJointUploadTime(up_time_millisecond);
}

void getArmState(UPLIMB_INPUT_INFO_STRUCT *recv_info) {
  auto& controller = getController();
  ::ul::math::Vector q(controller.getDof());
  ::ul::math::Vector qd(controller.getDof());
  ::ul::math::Vector tau(controller.getDof());
  ::std::vector<::std::uint16_t> statuswd(controller.getDof());
  ::std::vector<::std::uint8_t> mode(controller.getDof());
  unsigned long long int upTime;
//
  ::ul::math::Real up_time_millisecond = upTime / 1000000.0;
//  mode = controller.getJointMode();
//  q = controller.getJointPosition();
//  qd = controller.getJointVelocity();
//  controller.getJointStatusWord(statuswd);
//  tau = controller.getJointTorque();
  controller.getJointUploadTime(up_time_millisecond);
//
  for (int j = 0; j < controller.getDof(); ++j) {
//    recv_info->mode[j] = mode[j];
//    recv_info->act_pos[j] = q[j];
//    recv_info->act_vel[j] = qd[j];
////    recv_info->statuswd[j] = statuswd[j];
//    recv_info->act_tau[j] = tau[j];
  }
  recv_info->upTime = upTime;
//  recv_info->upTime = 0;
////  ::std::cout << "getArmState getJointMode: " << static_cast<int>(mode[0]) << std::endl;  // uint8_t会被当做char处理，从而打印出ASCII码
}

//void sendVarInfo(UPLIMB_VAR_OBSERVE *sendVar) {
//  ::ul::math::Vector ft_l(6), ft_r(6);
//  ::std::vector<::ul::math::Vector> data;
//
//  auto& controller = getController();
//  sendVar->var1[0] = controller.getTrajectoryCurrentStep();
//  sendVar->var1[1] = controller. getTrajectoryLength();
//  sendVar->var1[2] = static_cast<int>(controller.getRobotRunCommand());
//
//  // controller.getFTSensor(ft_l, ft_r);
//
//  ul::controller::interaction_control_buffer.get(data);
//
//  std::vector<ul::math::Vector6> cmd_ee_pose = getForwardKinematics(controller.getCommandJointPosition());
//
//  for (int j = 0; j < 6; ++j) {
//    sendVar->var6[j] = controller.getCommandCartesianPosition()[0][j];
//    sendVar->var6[j+6] = controller.getCommandCartesianPosition()[1][j];
//    // sendVar->var7[j] = controller.getCommandCartesianVelocity()[0][j];
//    // sendVar->var7[j+6] = controller.getCommandCartesianVelocity()[1][j];
//    // sendVar->var9[j] = controller->getForwardKinematics()[0][j];
//    // sendVar->var9[j+6] = controller->getForwardKinematics()[1][j];
//
//    // sendVar->var10[j] = cmd_ee_pose[0][j];
//    // sendVar->var10[j+6] = cmd_ee_pose[1][j];
//
//    // sendVar->var7[j] = ft_l[j];
//    // sendVar->var7[j+6] = ft_r[j];
//  }
//
//  for (int j = 0; j < controller.getDof(); ++j) {
//    sendVar->var8[j] = controller.getCommandJointAcceleration()[j];
//    // sendVar->var6[j] = data[4][j];  // act_vel/tau_e
//    // sendVar->var8[j] = data[1][j];  // FT
//    sendVar->var9[j] = data[0][j];  // ndo_est
//    // sendVar->var9[j] = data[1][j];  // tau_model/f_ee
//    // sendVar->var10[j] = data[0][j]; // tau_pid/Tau
//
//    // sendVar->var9[j] = controller.getGravityTorque()[j];
//    // sendVar->var8[j] = controller.get_servoJ_x_t()[j];
//    sendVar->var7[j] = controller.get_servoJ_x_t_f()[j];
//  }
//}
//
//void getArmVar(UPLIMB_VAR_OBSERVE *sendVar) {
//  auto& controller = getController();
//  for (::std::size_t j = 0; j < controller.getDof(); ++j) {
//    sendVar->var1[j] = 0;
//    sendVar->var2[j] = 0;
//    sendVar->var3[j] = 0;
//    sendVar->var4[j] = 0;
//    sendVar->var5[j] = 0;
//    sendVar->var6[j] = controller.getCommandJointAcceleration()[j];
//    sendVar->var7[j] = 0;
//    sendVar->var8[j] = 0;
//    sendVar->var9[j] = 0;
//    sendVar->var10[j] = 0;
//  }
//  sendVar->var1[0] = controller.getTrajectoryCurrentStep();
//  sendVar->var1[1] = controller.getTrajectoryLength();
//  sendVar->var1[2] = static_cast<int>(controller.getRobotRunCommand());
//
//  for (int j = 0; j < 6; ++j) {
//    sendVar->var7[j] = controller.getCommandCartesianPosition()[0][j];
//    // TODO: 程序启动时双臂末端姿态为NAN
//    if (::std::isnan(sendVar->var7[j])) sendVar->var7[j] = -9999.0;
//    sendVar->var7[j+6] = controller.getCommandCartesianPosition()[1][j];
//    if (::std::isnan(sendVar->var7[j+6])) sendVar->var7[j+6] = -9999.0;
//    sendVar->var8[j] = controller.getCommandCartesianVelocity()[0][j];
//    sendVar->var8[j+6] = controller.getCommandCartesianVelocity()[1][j];
//    sendVar->var9[j] = controller.getForwardKinematics()[0][j];
//    sendVar->var9[j+6] = controller.getForwardKinematics()[1][j];
//  }
//}
//
void sendDriverInfo(UPLIMB_OUTPUT_INFO_STRUCT *sendInfo) {
  auto& controller = getController();
    controller.run();
    for (::std::size_t j = 0; j < controller.getDof(); ++j) {
        sendInfo->cmd_pos[j] = controller.getCommandJointPosition()[j];
        sendInfo->cmd_vel[j] = controller.getCommandJointVelocity()[j];
        sendInfo->cmd_tau[j] = controller.getCommandJointTorque()[j];
        sendInfo->ctrlwd[j] = controller.getCommandJointCtrlwd()[j];
        sendInfo->mode_operation[j] = controller.getCommandJointModeOperation()[j];
        sendInfo->offset_torque[j] = controller.getCommandJointOffsetTorque()[j];
        sendInfo->offset_vel[j] = controller.getCommandJointOffsetVelocity()[j];
    }
    //////////////////////R180///////////////////////////////////
    // for (::std::size_t j = 0; j < 10; ++j) {
    //     sendInfo->cmd_pos[j] = controller.getCommandJointPosition()[j];
    //     sendInfo->cmd_vel[j] = controller.getCommandJointVelocity()[j];
    //     sendInfo->cmd_tau[j] = controller.getCommandJointTorque()[j];
    //     sendInfo->ctrlwd[j] = controller.getCommandJointCtrlwd()[j];
    //     sendInfo->mode_operation[j] = controller.getCommandJointModeOperation()[j];
    //     sendInfo->offset_torque[j] = controller.getCommandJointOffsetTorque()[j];
    //     sendInfo->offset_vel[j] = controller.getCommandJointOffsetVelocity()[j];
    // }

    // for (::std::size_t j = 14; j < 18; ++j) {
    //     sendInfo->cmd_pos[j] = controller.getCommandJointPosition()[j-4];
    //     sendInfo->cmd_vel[j] = controller.getCommandJointVelocity()[j-4];
    //     sendInfo->cmd_tau[j] = controller.getCommandJointTorque()[j-4];
    //     sendInfo->ctrlwd[j] = controller.getCommandJointCtrlwd()[j-4];
    //     sendInfo->mode_operation[j] = controller.getCommandJointModeOperation()[j-4];
    //     sendInfo->offset_torque[j] = controller.getCommandJointOffsetTorque()[j-4];
    //     sendInfo->offset_vel[j] = controller.getCommandJointOffsetVelocity()[j-4];
    // }

    // // sendInfo->mode_operation[16] = 8;
    // // sendInfo->offset_vel[16] = controller.getCommandJointVelocity()[12];

    // sendInfo->cmd_pos[18] = controller.getCommandJointPosition()[13];
    // sendInfo->cmd_vel[18] = controller.getCommandJointVelocity()[13];
    // sendInfo->cmd_tau[18] = controller.getCommandJointTorque()[13];
    // sendInfo->ctrlwd[18] = controller.getCommandJointCtrlwd()[13];
    // sendInfo->mode_operation[18] = controller.getCommandJointModeOperation()[13];
    // sendInfo->offset_torque[18] = controller.getCommandJointOffsetTorque()[13];
    // sendInfo->offset_vel[18] = controller.getCommandJointOffsetVelocity()[13];

    // // if (controller.getSafetyLock()) {
    // //   sendInfo->mode_operation[17] = 10;
    // //   sendInfo->cmd_tau[17] = controller.getCommandJointTorque()[13];
    // //   sendInfo->mode_operation[18] = 10;
    // //   sendInfo->cmd_tau[18] = 0.0;
    // // }
    // // else {
    //   sendInfo->mode_operation[17] = 9;
    //   sendInfo->offset_vel[17] = 0.0;
    //   sendInfo->mode_operation[18] = 9;
    //   sendInfo->offset_vel[18] = 0.0;
    // // }
    //////////////////////R180///////////////////////////////////
    //////////////////////I3///////////////////////////////////
    // for (::std::size_t j = 0; j < 4; ++j) {
    //     sendInfo->cmd_pos[j] = controller.getCommandJointPosition()[j];
    //     sendInfo->cmd_vel[j] = controller.getCommandJointVelocity()[j];
    //     sendInfo->cmd_tau[j] = controller.getCommandJointTorque()[j];
    //     sendInfo->ctrlwd[j] = controller.getCommandJointCtrlwd()[j];
    //     sendInfo->mode_operation[j] = controller.getCommandJointModeOperation()[j];
    //     sendInfo->offset_torque[j] = controller.getCommandJointOffsetTorque()[j];
    //     sendInfo->offset_vel[j] = controller.getCommandJointOffsetVelocity()[j];
    // }
    // for (::std::size_t j = 7; j < 11; ++j) {
    //     sendInfo->cmd_pos[j] = controller.getCommandJointPosition()[j-3];
    //     sendInfo->cmd_vel[j] = controller.getCommandJointVelocity()[j-3];
    //     sendInfo->cmd_tau[j] = controller.getCommandJointTorque()[j-3];
    //     sendInfo->ctrlwd[j] = controller.getCommandJointCtrlwd()[j-3];
    //     sendInfo->mode_operation[j] = controller.getCommandJointModeOperation()[j-3];
    //     sendInfo->offset_torque[j] = controller.getCommandJointOffsetTorque()[j-3];
    //     sendInfo->offset_vel[j] = controller.getCommandJointOffsetVelocity()[j-3];
    // }
    // for (::std::size_t j = 14; j < 16; ++j) {
    //     sendInfo->cmd_pos[j] = controller.getCommandJointPosition()[j-6];
    //     sendInfo->cmd_vel[j] = controller.getCommandJointVelocity()[j-6];
    //     sendInfo->cmd_tau[j] = controller.getCommandJointTorque()[j-6];
    //     sendInfo->ctrlwd[j] = controller.getCommandJointCtrlwd()[j-6];
    //     sendInfo->mode_operation[j] = controller.getCommandJointModeOperation()[j-6];
    //     sendInfo->offset_torque[j] = controller.getCommandJointOffsetTorque()[j-6];
    //     sendInfo->offset_vel[j] = controller.getCommandJointOffsetVelocity()[j-6];
    // }
    //////////////////////I3///////////////////////////////////
//    sendInfo->downTime = controller.getCommandDownTime()*1e6;
}


// 用于与用户端进行数据交互的单独线程
void serverRun(::std::atomic<bool>* keepRunning) {
  auto& controller = getController();
  controller.serverRun(keepRunning);
}

void getArmCMD(UPLIMB_OUTPUT_INFO_STRUCT *sendInfo) {
  auto& controller = getController();
  for (::std::size_t j = 0; j < controller.getDof(); ++j) {
    sendInfo->cmd_pos[j] = controller.getCommandJointPosition()[j];
    sendInfo->cmd_vel[j] = controller.getCommandJointVelocity()[j];
    sendInfo->cmd_tau[j] = controller.getCommandJointTorque()[j];
    sendInfo->ctrlwd[j] = controller.getCommandJointCtrlwd()[j];
    sendInfo->mode_operation[j] = controller.getCommandJointModeOperation()[j];
    sendInfo->offset_torque[j] = controller.getCommandJointOffsetTorque()[j];
    sendInfo->offset_vel[j] = controller.getCommandJointOffsetVelocity()[j];
  }
//  sendInfo->downTime = controller.getCommandDownTime()*1e6;
  sendInfo->downTime = 0;
}

//void getForceSensorInfo(ROBOT_FT_SENSOR_STATE_INFO_STRUCT *forceInfo) {
//  auto& controller = getController();
//  static ::ul::math::Vector ft_l(6), ft_r(6);
//
//  Eigen::Map<Eigen::VectorXd>(ft_l.data(), 6) <<
//    forceInfo->force[0].fx, forceInfo->force[0].fy, forceInfo->force[0].fz,
//    forceInfo->force[0].mx, forceInfo->force[0].my, forceInfo->force[0].mz;
//
//  Eigen::Map<Eigen::VectorXd>(ft_r.data(), 6) <<
//    forceInfo->force[1].fx, forceInfo->force[1].fy, forceInfo->force[1].fz,
//    forceInfo->force[1].mx, forceInfo->force[1].my, forceInfo->force[1].mz;
//
//  controller.setFTSensor(ft_l, ft_r);
//}
//
//bool setJointModeOperation(const RobotRunMode& run_mode, int arm_type) {
//  if(arm_type > 3) {
//    ::std::cout << RED << "[ERROR] arm_type is wrong!" << RESET << ::std::endl;
//    return false;
//  }
//  auto &controller_wrapper = getControllerWrapper();
//  if (!controller_wrapper.setJointModeOperation_adapter(static_cast<::ul::std17::RobotRunMode>(static_cast<int>(run_mode)), arm_type)) return false;
//  return true;
//}
//
//bool setControllerPara(const ControllerParasList& para_label, const Eigen::VectorXd& para, int arm_type) {
//  if(arm_type > 3) {
//    ::std::cout << RED << "[ERROR] arm_type is wrong!" << RESET << ::std::endl;
//    return false;
//  }
//  auto &controller_wrapper = getControllerWrapper();
//  ::ul::controller::ParasList para_label_temp = static_cast<::ul::controller::ParasList>(static_cast<int>(para_label));
//  return controller_wrapper.setControllerPara_adapter(para_label_temp, para, arm_type);
//}
//
//::Eigen::VectorXd getControllerPara(const ControllerParasList& para_label, int arm_type) {
//  auto &controller_wrapper = getControllerWrapper();
//  ::ul::controller::ParasList para_label_temp = static_cast<::ul::controller::ParasList>(static_cast<int>(para_label));
//  return controller_wrapper.getControllerPara_adapter(para_label_temp, arm_type);
//}
//
//bool setControllerMode(const ControllerMode &mode) {
//  auto &controller_wrapper = getControllerWrapper();
//  ::ul::controller::ControllerMode controller_mode_temp = static_cast<::ul::controller::ControllerMode>(static_cast<int>(mode));
//  return controller_wrapper.setControllerMode_adapter(controller_mode_temp);
//}
//
//bool getSafetyLock() {
//  auto& controller = getController();
//  return controller.getSafetyLock();
//}
//
//bool releaseSafetyLock() {
//  auto& controller = getController();
//  return controller.releaseSafetyLock();
//}
//
//void setLoadParas(::std::vector<double>& load_mass, ::std::vector<::Eigen::Matrix<double, 3, 1>>& load_barycenter_trans) {
//  auto& controller = getController();
//  controller.setLoadParas(load_mass, load_barycenter_trans);
//  return;
//}
//
//bool setJointPosition(const Eigen::VectorXd &q, int arm_type) {
//  auto &controller_wrapper = getControllerWrapper();
//  if (!controller_wrapper.setJointPosition_adapter(q, arm_type)) return false;
//  return true;
//}
//
//bool setJointTorque(const Eigen::VectorXd &tau, int arm_type) {
//  auto &controller_wrapper = getControllerWrapper();
//  if (!controller_wrapper.setJointTorque_adapter(tau, arm_type)) return false;
//  return true;
//}
//
//int getRobotCommand() {
//  auto& controller = getController();
//  return static_cast<int>(controller.getRobotRunCommand());
//}
//
//int isSingular(const Eigen::VectorXd &q, int arm_type) {
//  if (arm_type <= 0 || arm_type > 3) {
//    ::std::cout << RED << "[Error isSingular] arm_type is wrong!" << RESET << std::endl;
//    return -1;
//  }
//  auto &controller_wrapper = getControllerWrapper();
//  int isSingular = controller_wrapper.isSingular_adapter(q, arm_type);
//  if (isSingular == -1 ) return -1;
//  return (arm_type & isSingular);
//}
//
//int isSingular(int arm_type) {
//  if (arm_type <= 0 || arm_type > 3) {
//    ::std::cout << RED << "[Error isSingular] arm_type is wrong!" << RESET << std::endl;
//    return -1;
//  }
//  auto& controller = getController();
//  int isSingular = controller.isSingular();
//  if (isSingular == -1 ) return -1;
//  return (arm_type & isSingular);
//}
//
//bool loadControllerParas(std::string file_name) {
//  auto& controller = getController();
//  return controller.setControllerParas(file_name.c_str());
//}