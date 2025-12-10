/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-1-14
 * @Version       : 2.0.1
 * @File          : Controller.h
 ******************************************************************************
 */

#ifndef UL_SRC_UL_CONTROLLER_CONTROLLER_H_
#define UL_SRC_UL_CONTROLLER_CONTROLLER_H_
#include <ul/hal/Coach.h>
#include <ul/math/Pose2Transformation.h>
#include <ul/math/SShapeVelocity.h>
#include <ul/math/Spline.h>
#include <ul/math/Vector.h>
#include <ul/mdl/Dynamic.h>
#include <ul/std/common.h>
#include <ul/std/fileSave.h>
#include <ul/std/Socket.h>

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>
#ifdef REAL_MODE
//#include <alchemy/task.h>
//#include <alchemy/timer.h>
#endif

#include "CartesianMotion.h"
#include "JointsMotion.h"
#include "JointsSecurity.h"
#include "Teach.h"
#include "InteractionControl.h"

#include <iostream>
#include <Eigen/Dense>
#include <type_traits>
#include <memory>
#include <string>
#include <cmath>
#include <numeric>
#include <ul/math/Real.h>
#include <ul/math/Vector.h>
#include <ul/std/common.h>
#include <ul/std/DoubleBuffer.h>

#include "RobotFactory.h"


namespace ul {
namespace controller {
class Controller {
 public:
  Controller(const ::std::string& urdf_path);
  virtual ~Controller();

  void calculateDof();

  ::std::size_t getDof();
//
//  void setFTSensor(::ul::math::Vector& ft_l, ::ul::math::Vector& ft_r);
//
//  void getFTSensor(::ul::math::Vector& ft_l, ::ul::math::Vector& ft_r);
//
//  void getFTSensor_filtered(::ul::math::Vector& ft_l, ::ul::math::Vector& ft_r);
//
//  void getqd_qdd(::ul::math::Vector& qd, ::ul::math::Vector& qdd);
//
//  void getTau(::ul::math::Vector& tau_model, ::ul::math::Vector& tau_pid);
//
  // 获取URDF中加载的关节名称
  ::std::vector<::std::string> getJointName();
//
//  void cartesianInit(::ul::std17::CartesianConfig& c);
//
//  void setJointsNum(::std::vector<int> joints_num);
//
//  void setLoadParas(::std::vector<::ul::math::Real>& load_mass, ::std::vector<::ul::math::Vector3>& load_barycenter_trans);
//
  // 设置关节限位
  void setJointPositionLimit(const ::ul::math::Vector& lower_limit, const ::ul::math::Vector& upper_limit);
//  // 获取关节限位
//  void getJointPositionLimit(::ul::math::Vector& lower_limit, ::ul::math::Vector& upper_limit) const;
  // 设置位置下发警告阈值（cmd_q - act_q > pos_threshold）
  void setJointPositionThreshold(const double pos_threshold);
//  // 获取位置下发警告阈值（cmd_q - act_q > pos_threshold）
//  void getJointPositionThreshold(double& pos_threshold) const;
//
  void setJointFrictionParas(const double s_k, const ::ul::math::Vector& f_a, const ::ul::math::Vector& f_b, const ::ul::math::Vector& f_c);
//
//  bool setControllerParas(const char *file_name);
//
//  bool setControllerPara(::ul::controller::ParasList &para_label, ::ul::math::Vector &para);
//
//  bool getControllerPara(::ul::controller::ParasList &para_label, ::ul::math::Vector &para);
//
//  bool setControllerMode(::ul::controller::ControllerMode &mode);
//
//  bool getSafetyLock();
//
//  bool releaseSafetyLock();
//
//  // 获取期望的笛卡尔空间位置、速度、加速度
//  ::std::vector<::ul::math::Vector6> getCommandCartesianAcceleration();
//  ::std::vector<::ul::math::Vector6> getCommandCartesianPosition();
//  ::std::vector<::ul::math::Vector6> getCommandCartesianVelocity();
//
//  // 获取期望下载时间、关节加速度、控制字、模式、期望偏置力矩、期望偏置速度、期望位置、期望速度、期望力矩
//  ::ul::math::Real getCommandDownTime();
//
  ::ul::math::Vector getCommandJointAcceleration();
//
  ::std::vector<::std::uint16_t> getCommandJointCtrlwd();
//
  ::std::vector<::std::uint8_t> getCommandJointModeOperation();
//
  ::ul::math::Vector getCommandJointOffsetTorque();
//
  ::ul::math::Vector getCommandJointOffsetVelocity();
//
  ::ul::math::Vector getCommandJointPosition();
//
  ::ul::math::Vector getCommandJointVelocity();
//
  ::ul::math::Vector getCommandJointTorque();
//
//  // 计算笛卡尔空间末端位姿、速度
//  ::std::vector<::ul::math::Vector6> getEEAcc(const ::ul::math::Vector& q, const ::ul::math::Vector& qd, const ::ul::math::Vector& qdd);
//
//  ::std::vector<::ul::math::Vector6> getEEVel();
//
//  ::std::vector<::ul::math::Vector6> getEEVel(const ::ul::math::Vector& q, const ::ul::math::Vector& qd);
//
//  ::std::vector<::ul::math::Vector6> getForwardKinematics();
//
//  // 调用前需要匹配实物/ 仿真中的关节顺序重映射到pinocchio的关节顺序
//  ::std::vector<::ul::math::Vector6> getForwardKinematics(const ::ul::math::Vector& q);
//
//  int getInverseKinematics(const ::std::vector<::ul::math::Vector6>& pose,
//                            const ::std::vector<::ul::math::Real>& q7,
//                            ::std::vector<::std::vector<Eigen::VectorXd>>& q,
//                            ::std::vector<::std::vector<double>>& phi);
//
//  ::ul::math::Vector getGravityTorque();
//
//  ::ul::math::Vector getGravityTorque(const ::ul::math::Vector& q);
//
//  ::std::vector<::ul::math::Matrix> getJacobian();
//
//  ::std::vector<::ul::math::Matrix> getJacobian(const ::ul::math::Vector& q);
//
//  // 设置当前关节模式、当前关节位置、当前状态字、当前关节力矩、驱动器上传时间、当前关节速度
  void getJointMode(::std::vector<::std::uint8_t>& mode);
//
  void getJointPosition(::ul::math::Vector& q);
//
  void getJointStatusWord(::std::vector<::std::uint16_t>& statuswd);
//
  void getJointTorque(::ul::math::Vector& tau);
//
  void getJointUploadTime(::ul::math::Real& upTime);
//
  void getJointVelocity(::ul::math::Vector& qd);
//
//  ::ul::math::Vector getLastCommandJointPosition();
//
//  ::ul::math::Vector getLastCommandJointVelocity();
//
//  // 获取URDF中加载的机器人名称
  const ::std::string& getRobotName();
//
//  // 获取机器人运行指令
//  ::ul::std17::RobotCommand getRobotRunCommand() const;
//
//  int getTrajectoryCurrentStep() const;
//
//  int getTrajectoryLength() const;
//
//  // 获取当前关节模式、当前关节位置、当前关节力矩、当前关节速度
//  ::std::vector<uint8_t> getJointMode() const;
//
//  ::ul::math::Vector getJointPosition() const;
//
//  ::ul::math::Vector getJointTorque() const;
//
//  ::ul::math::Vector getJointVelocity() const;
//
//  bool goBack(::std::vector<::ul::math::Vector> q_home);
//
//  int isSingular();
//
//  int isSingular(const ::ul::math::Vector& q);
//
//  // 机器人运动指令: 关节空间位置运动
//  bool moveJ(const ::ul::math::Vector& q, double speed = 1.05, double acceleration = 1.4, bool asynchronous = false);
//  bool moveJ(const ::std::vector<::Eigen::Matrix<double, 6, 1>> &pose, const ::std::vector<math::Real>& q7, double speed=1.05, double acceleration=1.4, bool asynchronous=false);
//  bool moveJ(const ::ul::math::Vector& q, const math::Vector& qd, double freq, bool asynchronous);
//  bool moveJ(const ::std::vector<::std::vector<double>>& path, const double& time, bool asynchronous = false);
//  bool moveJ(const ::std::vector<::std::vector<double>>& path, const ::std::vector<double>& time, bool asynchronous = false);
//
//  // 机器人运动指令: 笛卡尔空间运动，目前仅考虑左手臂、右手臂，不考虑腰、颈
//  bool moveL(const ::std::vector<::ul::math::Vector6> &pose, const ::ul::math::Vector &ref_q, const double& speed, const double& acceleration, const bool& asynchronous);
//  bool moveL(const ::std::vector<::ul::math::Vector6>& pose, const double& speed = 0.25, const double& acceleration = 1.2, const bool& asynchronous = false);
//  bool moveL(const ::std::vector<::std::vector<double>>& path, const double& time, const bool& asynchronous = false);
//  bool moveL(const ::std::vector<::std::vector<double>>& path, const ::std::vector<double>& time, const bool& asynchronous);
//  bool moveNullSpace(const ::std::vector<::ul::math::Vector6> pose);
//
//  // TODO: time is not used
//  // 机器人运动指令: 关节空间速度控制
//  bool speedJ(const ::ul::math::Vector& qd, double acceleration = 0.5, double time = 0.0);
//
//  // TODO: time is not used
//  // 机器人运动指令: 笛卡尔空间速度控制
//  bool speedL(const ::std::vector<::ul::math::Vector6>& xd, const double& acceleration = 0.5, const double& time = 0.0);
//
//  bool speedStop(::ul::math::Real a = 10.0);
//
//  // bool loadCSV(const ::std::string &file_name);
//
//  // bool loadCSVData(const ::std::vector<::std::vector<::ul::math::Real>>& data);
//
//  // bool loadCSVData(std::vector<std::vector<ul::math::Real>> data);
//
//  bool loadCSVData(std::vector<std::vector<ul::math::Real>>&& data);
//
//  bool CSVTrajectoryMove(bool asynchronous);
//
//  bool teachMode(int arm_type);
//
//  bool endTeachMode(int arm_type);
//
//  bool setJointModeOperation(const ::std::vector<uint8_t>& mode_operation);
//
//  bool servoJ(const ::ul::math::Vector& q, const ::ul::math::Real& speed, const ::ul::math::Real& acceleration, const ::ul::math::Real& time, const ::ul::math::Real& lookahead_time, const ::ul::math::Real& gain);
//
//  bool servoL(const ::std::vector<::ul::math::Vector6>& pose, const ::ul::math::Real& speed, const ::ul::math::Real& acceleration, const ::ul::math::Real& time, const ::ul::math::Real& lookahead_time, const ::ul::math::Real& gain);
//

  /*!
   * 线程一：机器人运行函数
   */
  void run();
//
//  bool setJointPosition(const ::ul::math::Vector& q);
//
//  bool setJointTorque(const ::ul::math::Vector& tau);

  /*!
   * 线程二：Socket 运行函数
   * @param keepRunning: true 运行，false 停止
   */
  void serverRun(::std::atomic<bool>* keepRunning);
  void socketCmdHandler();
//
//  // test
//  std::vector<double> get_servoJ_x_t();
//  std::vector<double> get_servoJ_x_t_f();

 private:
  YAML::Node config_;

//  void fsm(::ul::std17::RobotCommand& cmd, ::ul::std17::RobotRunMode& mode, bool& first, const int& length, bool& is_complete, int& step);
//
  void readConfig();

  void createRobot();
//
//  void resetState(::ul::std17::RobotCommand& cmd, bool& is_complete, int& step, const ::std::string& task_name);
//
//  void sendCurrentJointPosition();
//
//  void unblockThread(const bool& asynchronous, ::std::unique_lock<std::mutex>& lock);
//
//  void unblockThread(::std::unique_lock<std::mutex>& lock);

  ::ul::mdl::Dynamic robot_;

  ::ul::hal::Coach driver_;

  mutable ::std::mutex mtx_;

  ::std::condition_variable cond_;

  bool first_no_cmd_ = true;

  bool init_pos_limit = false;

  bool is_collsion_flag_ = false;


  ::ul::std17::RobotCommand run_cmd_;

  ::ul::std17::RobotRunMode run_mode_;

  bool trajectory_planning_complete_;

  int trajectory_planning_length_;

  int trajectory_planning_step_;

  ::std::vector<int> body_trajectory_length_;

  ::std::vector<int> cartesian_trajectory_length_;

  InteractionControl interaction_control_;

  ::std::vector<int> offset_vel_on_;

  ::ul::std17::CartesianConfig cartesian_config_;

  ::ul::controller::ControllerMode controller_mode_ = ::ul::controller::ControllerMode::POSITION_CSP;

  bool protect_flag_vel_acc_ = false;

  bool emergency_stop_flag_ = false;
  
  bool collision_detection_on_;

  int arm_type_teach_ = 0;

  ::std::vector<int> joints_num_;

 private:
  ::std::size_t dof_;

  ::std::vector<::std::string> joint_names_;

  ::Eigen::VectorXi algorithm_idx_;

  ::std::vector<::Eigen::VectorXi> urdf_chain_idx_, alg_chain_idx_;

  static inline std::unique_ptr<Robot> robot_cfg_ = nullptr;

  ::ul::std17::ZmqCommandServer cmd_server_;  // 与用户通信的 zmq 类

  ::ul::std17::SocketCommand socket_cmd_;     // 与用户通信的 socket 数据结构体

  ::ul::std17::DoubleBuffer<UPLIMB_MAX_DIMENSION> double_buffer_;  // 双缓冲区类

  ::ul::std17::N2RBuffer cmd_paras_;          // 各种被调用接口的参数 数据结构体

};
}  // namespace controller
}  // namespace ul

#endif  // UL_SRC_UL_CONTROLLER_CONTROLLER_H_
