/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-2-21
 * @Version       : 0.0.1
 * @File          : AnalyticalInverseKinematics.h
 ******************************************************************************
 */

#ifndef UL_SRC_UL_MDL_ANALYTICALINVERSEKINEMATICS_H_
#define UL_SRC_UL_MDL_ANALYTICALINVERSEKINEMATICS_H_

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include "ul/math/Transform.h"

namespace ul {
namespace mdl {
class AnalyticalInverseKinematics {
 public:
  AnalyticalInverseKinematics();
  virtual ~AnalyticalInverseKinematics();
//   void init(const std::string &arm_type);
  void init(const std::string &arm_type, ::ul::math::Transform &T);
  void getForwardKinematics(const Eigen::VectorXd &q, Eigen::Matrix4d &T_ee);
  bool getInverseKinematics(const Eigen::Matrix4d &T_ee, const double &q7,
                            std::vector<Eigen::VectorXd> &q);
  bool returnArmAngle(std::vector<double> &arm_angle);
  bool getInverseKinematics(const Eigen::Matrix4d &T_ee, const double &q7,
                            std::vector<Eigen::VectorXd> &q,
                            std::vector<double> &arm_angle);

  ::std::vector<::Eigen::VectorXd> filterValidJointSolutions(const ::std::vector<::Eigen::VectorXd>& q_ik);
  // 计算解到关节限位的最小距离
  double computeMinDistanceToLimits(const Eigen::VectorXd& q);

  // 对有效解进行排序（按最小限位距离降序排列）
  void sortSolutionsByDistance(std::vector<Eigen::VectorXd>& valid_solutions);
  void sortSolutionsByDistance(::std::vector<Eigen::VectorXd>& vec, const Eigen::VectorXd& q);

  void setT_world_mdh_base(const ::ul::math::Transform &T);

 private:
  std::string arm_type_;
  Eigen::Vector3d l_bs_0_, l_se_3_, l_ew_4_, l_wt_7_;
  Eigen::VectorXd mdh_d_, mdh_a_, mdh_theta_, mdh_alpha_;
  Eigen::Matrix4d T_0_base_, T_7_MDH_URDF_, T_0_MDH_URDF_;
  double d_bs_, d_se_ , d_ew_, d_wt_;
  int dof_;
  std::vector<double> arm_angle_;
  ::Eigen::VectorXd joint_lower_, joint_upper_;

  void getJointRotation(const double &q, const double &d, const double &a, const double &alpha,
                        const double &theta, Eigen::Matrix3d &R);
  bool getJointTransformation(const double &q, const double &d, const double &a, const double &alpha, const double &theta,
                              Eigen::Matrix4d &T);
  void getSkewMatrix(const Eigen::Vector3d &v, Eigen::Matrix3d &S);
  void getInverseTransformation(const Eigen::Matrix4d &T, Eigen::Matrix4d &T_inv);
  bool getVirtualEE(const Eigen::Matrix4d &T_real, const double &q7, Eigen::Matrix4d &T_virtual);
  void getSWAxisFromTransformation(const Eigen::Matrix4d &T, Eigen::Vector3d &x_sw_0, Eigen::Matrix3d & R_7_0);
  bool getReferenceJointPos(const Eigen::Vector3d &x_sw_0,
                            std::vector<Eigen::Vector4d> &q_ref_list,
                            Eigen::Matrix3d &R_4_3);
  void getReferencePlaneRotation(const Eigen::Vector4d &q_ref, Eigen::Matrix3d &R_3_0_o);
  bool getAsBsCsMatrix(const Eigen::Vector3d &u_sw,
                       const Eigen::Matrix3d &R_3_0_o,
                       Eigen::Matrix3d &As,
                       Eigen::Matrix3d &Bs,
                       Eigen::Matrix3d &Cs);
  bool getAwBwCwMatrix(const Eigen::Matrix3d &R_4_3,
                       const Eigen::Matrix3d &R_7_0,
                       const Eigen::Matrix3d &As,
                       const Eigen::Matrix3d &Bs,
                       const Eigen::Matrix3d &Cs,
                       Eigen::Matrix3d &Aw,
                       Eigen::Matrix3d &Bw,
                       Eigen::Matrix3d &Cw) ;
  bool getArmAngle(const Eigen::Matrix3d &Aw, const Eigen::Matrix3d &Bw,
                   const Eigen::Matrix3d &Cw, const double &q7,
                   std::vector<double> &phi);
  bool checkArmAngle(const Eigen::Matrix3d &As, const Eigen::Matrix3d &Bs,
                     const Eigen::Matrix3d &Cs, const Eigen::Matrix3d &R_3_0_o,
                     const Eigen::Matrix3d &R_4_3, const Eigen::Matrix3d &u_sw_0_skew,
                     const Eigen::Vector3d &x_sw_0, const double &phi,
                     Eigen::Matrix3d &R_3_0);
  bool getPreviousThreeJoints(const Eigen::Matrix3d &R_3_0, Eigen::Vector3d &q_prev);
  bool getLastThreeJoints(const Eigen::Matrix3d &Aw, const Eigen::Matrix3d &Bw,
                          const Eigen::Matrix3d &Cw, const double &phi,
                          Eigen::Vector3d & q_last);
  bool checkMatrixError(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);
  bool checkVectorError(const Eigen::VectorXd& A, const Eigen::VectorXd& B);
  bool containsNaN(const Eigen::VectorXd& vec);
  bool checkResult(const Eigen::Matrix4d &T_ee,
                   const std::vector<Eigen::VectorXd> &q_ik,
                   std::vector<Eigen::VectorXd> &q_ik_valid,
                   std::vector<int> &valid_idx);
};
}  // namespace mdl
}  // namespace ul

#endif  // UL_SRC_UL_MDL_ANALYTICALINVERSEKINEMATICS_H_
