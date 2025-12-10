/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-2-20
 * @Version       : 0.0.1
 * @File          : CartesianMotion.h
 ******************************************************************************
 */

#ifndef UL_SRC_UL_CONTROLLER_CARTESIANMOTION_H_
#define UL_SRC_UL_CONTROLLER_CARTESIANMOTION_H_

#include <ul/hal/Coach.h>
#include <ul/math/Matrix.h>
#include <ul/math/Pose2Transformation.h>
#include <ul/math/Quaternion.h>
#include <ul/math/SShapeVelocity.h>
#include <ul/math/SingleSShapeVelocity.h>
#include <ul/math/Spline.h>
#include <ul/math/Vector.h>
#include <ul/mdl/Dynamic.h>
#include <ul/mdl/AnalyticalInverseKinematics.h>

#include <iostream>
#include <vector>
#include "pid_msd.h"

namespace ul {
namespace controller {
class CartesianMotion {
 public:
  CartesianMotion(::ul::hal::Coach& d, ::ul::mdl::Dynamic& r);

  ~CartesianMotion();

  void setCartesianConfig(::ul::std17::CartesianConfig& c);

  void setChainIndex(::std::vector<::Eigen::VectorXi>& urdf_chain_idx_, ::std::vector<::Eigen::VectorXi>& alg_chain_idx_);

  ::std::vector<::ul::math::Vector6> getCommandCartesianAcceleration();

  ::std::vector<::ul::math::Vector6> getCommandCartesianPosition();

  ::std::vector<::ul::math::Vector6> getCommandCartesianVelocity();

  ::ul::math::Vector getCommandJointAcceleration();

  ::ul::math::Vector getCommandJointPosition();

  ::ul::math::Vector getCommandJointVelocity();

  int getInverseKinematics(const ::std::vector<::ul::math::Vector6>& pose,
                            const ::std::vector<::ul::math::Real>& q7,
                            ::std::vector<::std::vector<Eigen::VectorXd>>& q,
                            ::std::vector<::std::vector<double>>& phi);

  ::std::vector<::ul::math::Vector> getSingularJointVelocity();

  int getTrajectoryLength();

  // 机器人运动指令: 笛卡尔空间运动，目前仅考虑左手臂、右手臂，不考虑腰、颈
  bool moveLNullSpace(const ::std::vector<::ul::math::Vector6>& pose, const ::ul::math::Vector& ref_q, const double& speed = 0.25, const double& acceleration = 1.2);

  bool moveL(const ::std::vector<::ul::math::Vector6>& pose, const double& speed = 0.25, const double& acceleration = 1.2);

  bool moveL(const ::std::vector<::ul::math::Vector6>& init_pose, const ::std::vector<::std::vector<double>>& path, const double& time);

  bool moveL(const ::std::vector<::ul::math::Vector6>& init_pose, const ::std::vector<::std::vector<double>>& path, const ::std::vector<double>& time);

  ::std::vector<::std::vector<::ul::math::Real>> moveNullSpace(const ::std::vector<::ul::math::Vector6> pose);

  // TODO: time is not used
  // 机器人运动指令: 笛卡尔空间速度控制
  bool speedL(const ::std::vector<::ul::math::Vector6> init_xd, const ::std::vector<::ul::math::Vector6> init_xdd, const ::std::vector<::ul::math::Vector6>& trg_xd, const double& acceleration = 0.5, const double& time = 0.0);
  bool servoL(const ::std::vector<::ul::math::Vector6>& init_x, const::std::vector<::ul::math::Vector6>& init_xd, const ::std::vector<::ul::math::Vector6>& pose, const ::ul::math::Real& speed, const ::ul::math::Real& acceleration, const ::ul::math::Real& time);
  bool servoL_resetParas(const ::ul::math::Real& lookahead_time, const ::ul::math::Real& gain);

  // 笛卡尔空间轨迹规划
  bool moveL_trajectory_nullspace(const int& step);

  bool moveL_trajectory(const int& step);

  bool moveL_path_trajectory(const int& step);

  bool moveNullSpace_trajectory(const int& step);

  bool speedL_trajectory(const int& step);

  bool servoL_trajectoryGenerated(const int &step);

 private:
  ::ul::hal::Coach& driver;
  ::ul::mdl::Dynamic& robot;
  ::std::size_t dof;

  ::ul::std17::CartesianConfig config; 

  ::std::vector<::Eigen::VectorXi> urdf_chain_idx_, alg_chain_idx_;

  // 笛卡尔空间轨迹规划时的比例
  ::std::vector<::ul::math::Real> cartesian_ratio_;

  ::std::vector<int> cartesian_trajectory_length_;

  ::ul::math::Vector cmd_q, cmd_qd, cmd_qdd, cmd_qddd;

  ::std::vector<::ul::math::Vector6> cmd_x, cmd_xd, cmd_xdd;

  ::std::vector<::ul::mdl::AnalyticalInverseKinematics> ik;

  int max_length;
  ::ul::math::Real max_time;

  // 笛卡尔空间位姿 doubleS 规划参数
  ::std::vector<::ul::math::SShapeVelocity<::ul::math::Real>> moveL_ssv_;
  // 笛卡尔空间起始终止的齐次变换对，vector大小为2表示左臂和右臂
  ::std::vector<::std::pair<::ul::math::Transform, ::ul::math::Transform>> moveL_data_;
  // 笛卡尔空间规划过程中的上一个时刻的左臂右臂位置
  ::std::vector<::ul::math::Vector3> moveL_last_position_;
  // 笛卡尔空间规划过程中的上一个时刻的左臂右臂四元数
  ::std::vector<::ul::math::Quaternion> moveL_last_quaternion_;

  // moveL path的规划参数
  ::std::vector<::ul::math::Real> moveL_path_;  // 一个轴的所有路径点
  // 笛卡尔空间位姿样条插值
  ::std::vector<::ul::math::Spline<::ul::math::Real>> moveL_spline_;
  // 笛卡尔空间样条插值时间
  ::std::vector<::ul::math::Real> moveL_spline_time_;

  // 零空间关节角记录
  ::std::vector<::ul::math::Vector> nullspace_q_;
  // 零空间关节角参考
  ::ul::math::Vector nullspace_q_ref_;

  // 笛卡尔空间单边 S型 速度规划参数，大小为2*6，2表示左右手臂，6表示笛卡尔空间自由度
  ::std::vector<::std::vector<::ul::math::SingleSShapeVelocity<::ul::math::Real>>> speedL_sv_;
  ::std::vector<::std::vector<::ul::math::Real>> speedL_ratio_;

// 机器人进入奇异位姿时各关节的速度
  ::std::vector<::ul::math::Vector> singular_qd;
  // 机器人奇异标志（非当前状态）
  ::std::vector<bool> isSingular;

  ::std::unique_ptr<PID_MSD> sys_servoL;
};
}  // namespace controller
}  // namespace ul

#endif  // UL_SRC_UL_CONTROLLER_CARTESIANMOTION_H_
