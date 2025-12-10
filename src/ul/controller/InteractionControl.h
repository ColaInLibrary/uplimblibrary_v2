/*
 * @Description:  
 * @Author: AN Hao, YUAN Hanqing
 * @Date: 2025-03-11 17:40:28
 * @LastEditors: YUAN Hanqing
 * @LastEditTime: 2025-03-18 09:37:51
 * @FilePath: /uplimlibrary/src/ul/controller/InteractionControl.h
 */

#ifndef UL_SRC_UL_INTERACTION_CONTROL_H_
#define UL_SRC_UL_INTERACTION_CONTROL_H_

#include <ul/hal/Coach.h>
#include <ul/mdl/Dynamic.h>
#include <ul/std/common.h>
#include <vector>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <ul/std/GaussianFilter.h>
#include <ul/std/Differentiator.h>
#include <ul/std/ButterworthFilter.h>
#include <ul/std/DebugShareBuffer.h>
#include "CartesianMotion.h"

namespace ul {
namespace controller {
enum class ParasList { TORQUE_LIMIT = 0, NDO_GAIN = 1, KP, KD, K_TEACH, D_TEACH };  
enum class ControllerMode { POSITION_CSP = 0, POSITION_CST = 1, IMPEDANCE };  
extern ::ul::std17::DebugShareBuffer interaction_control_buffer;
class InteractionControl {
 public:

  InteractionControl(::ul::hal::Coach& d, ::ul::mdl::Dynamic& r, const ::std::size_t& dof);

  ~InteractionControl();
  
  void setFTSensor(::ul::math::Vector& ft_l, ::ul::math::Vector& ft_r);

  void getFTSensor(::ul::math::Vector& ft_l, ::ul::math::Vector& ft_r);

  ::ul::math::Vector getTauExt();

  ::Eigen::Array<bool, Eigen::Dynamic, 1> getJointCollisionFlag();

  void update();

  void trajectoryTrackingController();

  void impedanceController(::ul::math::Vector &kp, ::ul::math::Vector &kd);

  void impedanceController();

  bool protectedMode(double a_max, double delta_p_min);
  
  void impedanceController_protected();

  ::ul::math::Vector NDO();    

  ::std::vector<::ul::math::Vector> tau2fee(bool isArm, ::std::vector<::ul::math::Vector> tau);

  ::std::vector<::ul::math::Vector> fee2tau(bool isArm, ::std::vector<::ul::math::Vector> fee);

  void setCartesianConfig(::ul::std17::CartesianConfig& c);  

  void setChainIndex(::std::vector<::Eigen::VectorXi>& urdf_chain_idx_, ::std::vector<::Eigen::VectorXi>& alg_chain_idx_);  

  void setLoadParas(::std::vector<::ul::math::Real>& load_mass, ::std::vector<::ul::math::Vector3>& load_barycenter_trans); 

  bool setControllerParas(const char *file_name, bool printout);   

  bool setControllerPara(::ul::controller::ParasList &para_label, ::ul::math::Vector &para, bool printout);

  bool getControllerPara(::ul::controller::ParasList &para_label, ::ul::math::Vector &para);

  void setJointPositionLimit(const ::ul::math::Vector& lower_limit, const ::ul::math::Vector& upper_limit);

  bool teachMode(::ul::std17::RobotCommand& cmd, int arm_type);   

  bool endTeachMode(::ul::std17::RobotCommand& cmd, int arm_type);     

  bool teachTorqueGenerate();  

 private:
  ::ul::hal::Coach& driver;

  ::ul::mdl::Dynamic& robot;

  ::std::size_t dof;

  ::ul::math::Real sampling_period;

  ::ul::math::Vector cmd_q, cmd_qd, cmd_qdd, cmd_qddd;

  ::ul::math::Vector act_q, act_qd, act_qdd, act_qddd;

  ::ul::math::Vector act_tau, cmd_tau;

  ::ul::math::Vector tau_model, tau_pid;

  ::ul::math::Vector act_tau_l, cmd_tau_l, act_tau_r, cmd_tau_r;

  ::ul::math::Vector FT_sensor_l, FT_sensor_r;

  ::ul::math::Vector FT_sensor_l_filtered, FT_sensor_r_filtered;

  ::ul::math::Vector FT_sensor_l_initial, FT_sensor_r_initial;

  ::ul::math::Matrix M_d, invM_d;  // 期望值

  ::ul::math::Vector tau_c_d, tau_g_d, tau_f_d;  // 期望值

  ::ul::math::Matrix M, invM;  // 实际值

  ::ul::math::Vector tau_c, tau_g, tau_f; // 实际值

  ::std::vector<::ul::math::Matrix> J_arm, J_chain;

  ::ul::math::Vector tau_est; // 关节外力矩估计值

  ::ul::math::Vector tau_ext; // 关节外力矩估计值(阻抗)

  ::ul::math::Vector ndo_p_temp, ndo_d_hat_temp, ndo_gain;  // ndo观测器参数

  ::ul::math::Vector Kp, Kd, K_impdc, D_impdc, K_teach, D_teach, K_protect, D_protect;  // 力控参数

  ::ul::math::Vector cmd_tau_filtered, act_tau_filtered, qd_filtered;

  ::ul::math::Vector act_qd_d, act_qdd_d, cmd_qd_d, cmd_qdd_d;

  ::ul::math::Vector cmd_tau_filtered_l, qd_filtered_l, cmd_tau_filtered_r, qd_filtered_r;

  ::std::vector<::std::uint8_t> run_mode; 

  ::ul::math::Vector joint_torque_limit; // 关节力矩输出限制

  ::ul::math::Vector lower_pos_limit, upper_pos_limit;  // 关节限位

  ::ul::math::Vector lower_pos_limit_protect, upper_pos_limit_protect;  // 保护限位

  ::ul::math::Vector cmd_q_prev, cmd_q_prev_protect;

  bool joint_tau_limit_flag;

  ::ul::math::Vector joint_collision_threshold; // 碰撞检测关节力矩阈值

  Eigen::Array<bool, Eigen::Dynamic, 1> joint_collision_flag; // 关节碰撞标志

  ::ul::math::Vector delta_v;

  std::unique_ptr<::ul::std17::GaussianFilter> ft_filter, act_qd_filter, act_qdd_filter, cmd_qd_filter, cmd_qdd_filter, cmd_tau_filter, act_tau_filter, cmd_tau_filter_r;
  // std::unique_ptr<::ul::std17::ButterworthFilter> ft_filter, act_qd_filter, act_qdd_filter, cmd_qd_filter, cmd_qdd_filter, cmd_tau_filter, act_tau_filter, cmd_tau_filter_r;

  std::unique_ptr<::ul::std17::Differentiator> act_q_diff, cmd_q_diff;

  ::ul::std17::CartesianConfig cartesian_config;

  ::ul::math::Vector3 g_world;

  ::std::vector<::ul::math::Real> load_mass;

  ::std::vector<::ul::math::Vector3> g_tool, load_barycenter_trans;

  ::std::vector<::ul::math::Vector6> gravity_tool;

  ::std::vector<::ul::math::Vector> tau_tool;

  ::std::vector<::Eigen::VectorXi> urdf_chain_idx_, alg_chain_idx_;
};
}  // namespace controller
}  // namespace ul

#endif  // UL_SRC_UL_CONTROLLER_JOINTSSECURITY_H_
