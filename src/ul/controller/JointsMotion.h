/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-2-19
 * @Version       : 0.0.1
 * @File          : JointsMotion.h
 ******************************************************************************
 */

#ifndef UL_SRC_UL_CONTROLLER_JOINTSMOTION_H_
#define UL_SRC_UL_CONTROLLER_JOINTSMOTION_H_

#include <ul/math/Matrix.h>
#include <ul/math/Quaternion.h>
#include <ul/math/SShapeVelocity.h>
#include <ul/math/Spline.h>
#include <ul/math/Vector.h>
#include <ul/mdl/Dynamic.h>
#include <vector>
#include <ul/math/SingleSShapeVelocity.h>
#include <ul/math/Spline.h>
//#include <ul/std/fileSave.h>

#include <iostream>
#include "pid_msd.h"


namespace ul {
namespace controller {
class JointsMotion {
 public:

  JointsMotion(const ::std::size_t& dof);

  ~JointsMotion();

  struct speedStop_data {
    ::ul::math::Real time;
    ::ul::math::Real acc;
    ::ul::math::Real vel;
  };

  int getTrajectoryLength();

  // 机器人运动指令: 关节空间位置运动
  bool moveJ(const ::ul::math::Vector &q, const double& speed=1.05, const double& acceleration=1.4);
  bool moveJ_Spline(const ::ul::math::Vector &q, const ::ul::math::Vector &qd, const double& freq);
  bool moveJ(const ::std::vector<::std::vector<double>> &path, const double& time);
  bool moveJ(const ::std::vector<::std::vector<double>> &path, const ::std::vector<double>& time);

  bool speedJ(const ::ul::math::Vector& qd, const double& acceleration = 0.5, const double& time = 0.0);

  bool speedStop(const ::ul::math::Real& a = 10);

  bool loadCSVData(::std::vector<::std::vector<::ul::math::Real>>&& data);

  bool CSVTrajectoryMove();

  bool servoJ(const ::ul::math::Vector& q, const ::ul::math::Real& speed, const ::ul::math::Real& acceleration, const ::ul::math::Real& time);
  
  bool servoJ_resetParas(const ::ul::math::Real& lookahead_time, const ::ul::math::Real& gain);

  bool moveJ_trajectory(const int& step);

  bool moveJ_path_trajectory(const int& step);

  bool moveJ_spline_trajectory(const int& step);
  
  bool speedJ_trajectory(const int& step);

  bool speed_stop_trajectory(const int& step);

  bool csv_trajectory(const int& step);

  bool servoJ_trajectoryGenerated(const int &step);

  // test
  std::vector<double> get_servoJ_x_t();
  std::vector<double> get_servoJ_x_t_f();

 private:
  ::ul::math::Vector cmd_q, cmd_qd, cmd_qdd, cmd_qddd;

  ::ul::math::Vector cmd_q_last, cmd_qd_last, cmd_qdd_last;

  ::std::size_t dof;

  ::ul::math::Real dt;

  // 关节空间轨迹规划时的比例
  ::std::vector<::ul::math::Real> joint_ratio;

  ::std::vector<int> joint_trajectory_length;

  int max_length;

  // 关节空间位置 doubleS 规划参数
  ::std::vector<::ul::math::SShapeVelocity<::ul::math::Real>> moveJ_ssv;

  ::std::vector<::ul::math::Real> moveJ_path;  // 一个轴的所有路径点

  ::std::vector<::ul::math::Spline<::ul::math::Real>> moveJ_spline;

  // 关节空间位置样条插值时间
  ::std::vector<::ul::math::Real> moveJ_spline_time;

  // 关节空间单边 S型 速度规划参数
  ::std::vector<::ul::math::SingleSShapeVelocity<::ul::math::Real>> speedJ_sv;

  // 关节空间 速度停止 规划参数
  ::std::vector<speedStop_data> speedStop_data;

  // 关节空间 单点moveJ_spline
  ::std::vector<::ul::math::Polynomial<::ul::math::Real>> moveJ_point_spline;

  // 关节空间 csv加载的轨迹数据
  ::std::vector<::std::vector<::ul::math::Real>> csv_data;

  ::std::unique_ptr<PID_MSD> sys_servoJ;
};
}  // namespace controller
}  // namespace ul

#endif  // UL_SRC_UL_CONTROLLER_JOINTSMOTION_H_
