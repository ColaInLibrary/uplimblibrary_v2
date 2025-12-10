/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-2-21
 * @Version       : 0.0.1
 * @File          : AnalyticalInverseKinematics.cpp
 ******************************************************************************
 */

#include "AnalyticalInverseKinematics.h"

namespace ul {
namespace mdl {
AnalyticalInverseKinematics::AnalyticalInverseKinematics() : joint_lower_(7), joint_upper_(7) {}

AnalyticalInverseKinematics::~AnalyticalInverseKinematics() {}

void AnalyticalInverseKinematics::init(const std::string &arm_type, ::ul::math::Transform &T) {
  dof_ = 7;
  arm_type_ = arm_type;
  mdh_d_.resize(dof_);
  mdh_a_.resize(dof_);
  mdh_theta_.resize(dof_);
  mdh_alpha_.resize(dof_);
  l_bs_0_.resize(3);
  l_wt_7_.resize(3);
  l_se_3_.resize(3);
  l_ew_4_.resize(3);
  if (arm_type_ == "LEFT") {
    mdh_d_ << 0.075202, 0, -0.24, 0.0, -0.207, 0, 0;  // new_arm
    mdh_a_.setZero();
    mdh_a_(6) = -0.063;  // new_arm
    mdh_theta_ << 0, -110 * M_PI / 180, -M_PI / 2, 0, 0, -M_PI / 2, -M_PI;  // new_arm
    mdh_alpha_ << 0, -M_PI_2, -M_PI_2, -M_PI_2, M_PI_2, -M_PI_2, -M_PI_2;
    // T_0_base_ <<                          0, 1,                         0,       0,
    //              -sin(20.0 * M_PI / 180), 0, cos(20.0 * M_PI / 180), 0.15869,
    //               cos(20.0 * M_PI / 180), 0, sin(20.0 * M_PI / 180), 0.29644+0.0694,
    //                                       0, 0,                         0,        1;
    d_bs_ = 0.075202;
    joint_lower_ << -3.0, 0.0, -2.9, -2.0, -2.9, -1.5, -1.5;
    joint_upper_ << 1.0, -3.0, 2.9, 0.0, 2.9, 1.5, 1.5;
  } else if (arm_type_ == "RIGHT") {
    mdh_d_ << -0.075202, 0, -0.24, 0.0, -0.207, 0, 0;
    mdh_a_.setZero();
    mdh_a_(6) = -0.063;
    mdh_theta_ << 0.0, -70 * M_PI / 180, -M_PI / 2, 0, 0, -M_PI / 2, -M_PI;
    mdh_alpha_ << 0, -M_PI_2, -M_PI_2, -M_PI_2, M_PI_2, -M_PI_2, -M_PI_2;
    
    // T_0_base_ <<                         0, 1,                         0,        0,
    //              sin(20.0 * M_PI / 180), 0, cos(20.0 * M_PI / 180), -0.15869,
    //              cos(20.0 * M_PI / 180), 0, -sin(20.0 * M_PI / 180), 0.29644+0.0694,
    //                                      0, 0,                          0,       1;
    d_bs_ = -0.075202;
    joint_lower_ << -3.0, -3.0, -2.9, -2.0, -2.9, -1.5, -1.5;
    joint_upper_ << 1.0, 0.0, 2.9, 0.0, 2.9, 1.5, 1.5;
  }
  T_0_MDH_URDF_ <<  0,  1,  0,  0,
                    0,  0,  1,  0,
                    1,  0,  0,  0,
                    0,  0,  0,  1;
  T_0_base_ = T * T_0_MDH_URDF_;  
  // std::cout << "init_T[" << arm_type << "]: \n" << T_0_base_ << std::endl;              
  T_7_MDH_URDF_ << 0, 0, -1, 0.060997, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
  d_se_ = -0.24;
  d_ew_ = -0.207;
  d_wt_ = 0;
  l_bs_0_ << 0, 0, d_bs_;
  l_se_3_ << 0, 0, d_se_;
  l_ew_4_ << 0, -d_ew_, 0;
  l_wt_7_ << 0, 0, d_wt_;
}

void AnalyticalInverseKinematics::getJointRotation(const double &q, const double &d, const double &a, const double &alpha, const double &theta,
                                                   Eigen::Matrix3d &R) {
  double th = q + theta;
  R << cos(th), -sin(th), 0,
      cos(alpha) * sin(th), cos(alpha) * cos(th), -sin(alpha),
      sin(alpha) * sin(th), sin(alpha) * cos(th), cos(alpha);
}

bool AnalyticalInverseKinematics::getJointTransformation(const double &q, const double &d, const double &a, const double &alpha, const double &theta,
                                                         Eigen::Matrix4d &T) {
  double th = q + theta;
  T << cos(th), -sin(th), 0, a,
      cos(alpha) * sin(th), cos(alpha) * cos(th), -sin(alpha), -d * sin(alpha),
      sin(alpha) * sin(th), sin(alpha) * cos(th),  cos(alpha), d * cos(alpha),
      0, 0, 0, 1;
  return true;
}

void AnalyticalInverseKinematics::getSkewMatrix(const Eigen::Vector3d &v, Eigen::Matrix3d &S) { S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0; }

void AnalyticalInverseKinematics::getInverseTransformation(const Eigen::Matrix4d &T, Eigen::Matrix4d &T_inv) {
  T_inv.setIdentity();
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  T_inv.block<3, 3>(0, 0) = R.transpose();
  T_inv.block<3, 1>(0, 3) = -R.transpose() * T.block<3, 1>(0, 3);
}

bool AnalyticalInverseKinematics::getVirtualEE(const Eigen::Matrix4d &T_real, const double &q7, Eigen::Matrix4d &T_virtual) {
  Eigen::Matrix4d invT0, invT7;
  getInverseTransformation(T_0_base_, invT0);
  getInverseTransformation(T_7_MDH_URDF_, invT7);
  Eigen::Matrix4d T_real_MDH = invT0 * T_real * invT7;
  Eigen::Matrix4d T67;
  getJointTransformation(q7, mdh_d_[6], mdh_a_[6], mdh_alpha_[6], mdh_theta_[6], T67);
  Eigen::Vector3d p07, p06, p67;
  Eigen::Matrix3d R07, R67;
  p07 = T_real_MDH.block<3, 1>(0, 3);
  R07 = T_real_MDH.block<3, 3>(0, 0);
  p67 = T67.block<3, 1>(0, 3);
  R67 = T67.block<3, 3>(0, 0);
  p06 = p07 - R07 * R67.transpose() * p67;
  T_virtual.setIdentity();
  T_virtual.block<3, 1>(0, 3) = p06;
  T_virtual.block<3, 3>(0, 0) = R07;
  return true;
}

void AnalyticalInverseKinematics::getSWAxisFromTransformation(const Eigen::Matrix4d &T, Eigen::Vector3d &x_sw_0, Eigen::Matrix3d &R_7_0) {
  Eigen::Vector3d x_7_0;
  x_7_0 = T.block(0, 3, 3, 1);
  R_7_0 = T.block(0, 0, 3, 3);
  x_sw_0 = x_7_0 - l_bs_0_ - R_7_0 * l_wt_7_;
}

bool AnalyticalInverseKinematics::getReferenceJointPos(const Eigen::Vector3d &x_sw_0, std::vector<Eigen::Vector4d> &q_ref_list, Eigen::Matrix3d &R_4_3) {
  q_ref_list.resize(2);
  // Step1: 求关节4的角度
  double cos_q4 = (x_sw_0.norm() * x_sw_0.norm() - d_se_ * d_se_ - d_ew_ * d_ew_) / (2 * d_se_ * d_ew_);
  //  assert(cos_q4 <= 1);
  if (abs(cos_q4) > 1) {
    std::cout << arm_type_ << " [Error] Joint[3] is out of range!" << std::endl;
    return false;
  }
  double q4 = acos(cos_q4);
  if (q4 > 0) q4 = -q4;
#ifdef DISPLAY
  std::cout << "cos_q4: " << cos_q4 << std::endl;
#endif

  // Step2: 求参考平面的角度
  double s4, c4;
  s4 = sin(q4);
  c4 = cos(q4);
  getJointRotation(q4, mdh_d_[3], mdh_a_[3], mdh_alpha_[3], mdh_theta_[3], R_4_3);
  double a, b;
  if (arm_type_ == "LEFT") {
    a = d_se_ + d_ew_ * c4;
  } else if (arm_type_ == "RIGHT") {
    a = -(d_se_ + d_ew_ * c4);
  }
  b = d_ew_ * s4;
  double R = sqrt(a * a + b * b);
  if (fabs(x_sw_0(2) / R) > 1)  // 真没有解
    return false;

  double q3_ref = M_PI_2;
  double psi = atan2(b, a);
  double tt_temp = asin(x_sw_0[2] / R);
  std::vector<double> temp(2);
  temp[0] = tt_temp - psi;
  if (temp[0] >= 0) {
    temp[1] = M_PI - tt_temp - psi;
  } else {
    temp[1] = -M_PI - tt_temp - psi;
  }
  double q2o;

  for (int i = 0; i < 2; ++i) {
    if (arm_type_ == "LEFT") {
      q2o = M_PI / 9 - temp[i];
      b = cos(temp[i]) * (d_se_ + d_ew_ * c4) - d_ew_ * sin(temp[i]) * sin(q3_ref) * s4;
    } else if (arm_type_ == "RIGHT") {
      q2o = temp[i] - M_PI / 9;
      b = cos(temp[i]) * (d_se_ + d_ew_ * c4) + d_ew_ * sin(temp[i]) * sin(q3_ref) * s4;
    } else {
      std::cout << arm_type_ << " [Error] Arm type is not defined!" << std::endl;
      return false;
    }
    a = d_ew_ * cos(q3_ref) * s4;
    double sin_q1o = (b * x_sw_0[1] - a * x_sw_0[0]) / (a * a + b * b);
    double cos_q1o = (a * x_sw_0[1] + b * x_sw_0[0]) / (a * a + b * b);
    double q1o = atan2(sin_q1o, cos_q1o);
    q_ref_list[i] << q1o, q2o, q3_ref, q4;
#ifdef DISPLAY
    Eigen::Vector3d x_sw_0_check;
    x_sw_0_check[0] = cos(q1o) * b - a * sin(q1o);
    x_sw_0_check[1] = sin(q1o) * b + a * cos(q1o);
    x_sw_0_check[2] = -(d_se_ + d_ew_ * c4) * sin(temp[i]) + d_ew_ * cos(temp[i]) * sin(q3_ref) * s4;
    std::cout << x_sw_0.transpose() << std::endl;
    std::cout << x_sw_0_check.transpose() << std::endl;
#endif
  }
  return true;
}

void AnalyticalInverseKinematics::getReferencePlaneRotation(const Eigen::Vector4d &q_ref, Eigen::Matrix3d &R_3_0_o) {
  Eigen::Matrix3d R1o, R2o, R3o;
  getJointRotation(q_ref[0], mdh_d_[0], mdh_a_[0], mdh_alpha_[0], mdh_theta_[0], R1o);
  getJointRotation(q_ref[1], mdh_d_[1], mdh_a_[1], mdh_alpha_[1], mdh_theta_[1], R2o);
  getJointRotation(q_ref[2], mdh_d_[2], mdh_a_[2], mdh_alpha_[2], mdh_theta_[2], R3o);
  R_3_0_o = R1o * R2o * R3o;
}

bool AnalyticalInverseKinematics::getAsBsCsMatrix(const Eigen::Vector3d &u_sw, const Eigen::Matrix3d &R_3_0_o, Eigen::Matrix3d &As, Eigen::Matrix3d &Bs,
                                                  Eigen::Matrix3d &Cs) {
  Eigen::Matrix3d u_sw_0_skew;
  getSkewMatrix(u_sw, u_sw_0_skew);
  As = u_sw_0_skew * R_3_0_o;
  Bs = -u_sw_0_skew * u_sw_0_skew * R_3_0_o;
  Cs = u_sw * u_sw.transpose() * R_3_0_o;
  return true;
}

bool AnalyticalInverseKinematics::getAwBwCwMatrix(const Eigen::Matrix3d &R_4_3, const Eigen::Matrix3d &R_7_0, const Eigen::Matrix3d &As,
                                                  const Eigen::Matrix3d &Bs, const Eigen::Matrix3d &Cs, Eigen::Matrix3d &Aw, Eigen::Matrix3d &Bw,
                                                  Eigen::Matrix3d &Cw) {
  Aw = R_4_3.transpose() * As.transpose() * R_7_0;
  Bw = R_4_3.transpose() * Bs.transpose() * R_7_0;
  Cw = R_4_3.transpose() * Cs.transpose() * R_7_0;
  return true;
}

/**
 * @brief 求臂角，输出为2个臂角
 * @param Aw 输入
 * @param Bw 输入
 * @param Cw 输入
 * @param q7 输入
 * @param phi 输出
 * @return 成功为true，失败为false
 */
bool AnalyticalInverseKinematics::getArmAngle(const Eigen::Matrix3d &Aw, const Eigen::Matrix3d &Bw, const Eigen::Matrix3d &Cw, const double &q7,
                                              std::vector<double> &phi) {
  phi.clear();
  double A, B, C;
  A = Aw(1, 1) + Aw(1, 0) * tan(q7);
  B = Bw(1, 1) + Bw(1, 0) * tan(q7);
  C = Cw(1, 1) + Cw(1, 0) * tan(q7);
  double alpha = atan2(B, A);
  double R = sqrt(pow(A, 2) + pow(B, 2));
  if (R == 0) {
    std::cout << arm_type_ << " [Error] R is zero! Please check the input!" << std::endl;
    return false;
  }
  if (fabs(-C / R) > 1) {
    //    std::cout << arm_type_
    //              << " [Error] getIK solution failed. Please change another q7!"
    //              << std::endl;
    return false;
  }
  double psi0 = asin(-C / R);
  if (psi0 >= 0) {
    phi.push_back(psi0 - alpha);
    phi.push_back(M_PI - psi0 - alpha);
  } else {
    phi.push_back(psi0 - alpha);
    phi.push_back(-M_PI - psi0 - alpha);
  }
  // 检查一下是否正确
  for (int i = 0; i < 2; ++i) {
    double temp = A * sin(phi[i]) + B * cos(phi[i]) + C;
    if (fabs(temp) > 1e-6) {
      std::cout << arm_type_ << " [Error] getIK solution failed." << std::endl;
      return false;
    }
  }
  return true;
}

bool AnalyticalInverseKinematics::checkArmAngle(const Eigen::Matrix3d &As, const Eigen::Matrix3d &Bs, const Eigen::Matrix3d &Cs, const Eigen::Matrix3d &R_3_0_o,
                                                const Eigen::Matrix3d &R_4_3, const Eigen::Matrix3d &u_sw_0_skew, const Eigen::Vector3d &x_sw_0,
                                                const double &phi, Eigen::Matrix3d &R_3_0) {
  Eigen::Matrix3d R_phi_0;
  R_phi_0 = Eigen::Matrix3d::Identity() + sin(phi) * u_sw_0_skew + (1 - cos(phi)) * u_sw_0_skew * u_sw_0_skew;
  R_3_0 = R_phi_0 * R_3_0_o;

  Eigen::Matrix3d R_3_0_check;
  R_3_0_check = As * sin(phi) + Bs * cos(phi) + Cs;
  if (!checkMatrixError(R_3_0, R_3_0_check)) return false;

  Eigen::Vector3d x_sw_3, x_sw_0_check;
  x_sw_3 = l_se_3_ + R_4_3 * l_ew_4_;
  x_sw_0_check = R_3_0 * x_sw_3;
  if (!checkMatrixError(x_sw_0, x_sw_0_check)) return false;

  return true;
}

bool AnalyticalInverseKinematics::getPreviousThreeJoints(const Eigen::Matrix3d &R_3_0, Eigen::Vector3d &q_prev) {
  q_prev(2) = atan2(R_3_0(2, 0), R_3_0(2, 1));  // q3
  q_prev(0) = atan2(R_3_0(1, 2), R_3_0(0, 2));  // q1
  q_prev(1) = -999;

  double sin_q2;
  if (arm_type_ == "LEFT") sin_q2 = R_3_0(2, 2);
  if (arm_type_ == "RIGHT") sin_q2 = -R_3_0(2, 2);

  if (fabs(sin_q2) > 1) {
    //    std::cout << arm_type_
    //              << " [Error] getIK solution failed. Please change another q7!"
    //              << std::endl;
    return false;
  }
  std::vector<double> temp(2);
  temp[0] = asin(sin_q2);
  if (temp[0] >= 0) {
    temp[1] = M_PI - temp[0];
  } else {
    temp[1] = -M_PI - temp[0];
  }

  for (int i = 0; i < 2; ++i) {
    double err1 = fabs(R_3_0(2, 0) - cos(temp[i]) * sin(q_prev[2]));
    double err2 = fabs(R_3_0(2, 1) - cos(temp[i]) * cos(q_prev[2]));
    if (err1 > 1e-5 || err2 > 1e-5) {
      ;
    } else {
      if (arm_type_ == "LEFT") {
        q_prev[1] = M_PI / 9 - temp[i];
      } else {
        q_prev[1] = -M_PI / 9 + temp[i];
      }
    }
  }
  if (q_prev[1] == -999) return false;
  return true;
}

bool AnalyticalInverseKinematics::getLastThreeJoints(const Eigen::Matrix3d &Aw, const Eigen::Matrix3d &Bw, const Eigen::Matrix3d &Cw, const double &phi,
                                                     Eigen::Vector3d &q_last) {
  Eigen::Matrix3d R_7_4 = Aw * sin(phi) + Bw * cos(phi) + Cw;
  q_last(0) = atan2(R_7_4(2, 2), R_7_4(0, 2));   // q5
  q_last(2) = atan2(-R_7_4(1, 1), R_7_4(1, 0));  // q7
  std::vector<double> temp(2);
  temp[0] = asin(R_7_4(1, 2));
  if (temp[0] >= 0) {
    temp[1] = M_PI - temp[0];
  } else {
    temp[1] = -M_PI - temp[0];
  }

  q_last(1) = -999;
  for (int i = 0; i < 2; ++i) {
    double err1 = fabs(R_7_4(2, 2) - cos(temp[i]) * sin(q_last(0)));
    double err2 = fabs(R_7_4(0, 2) - cos(temp[i]) * cos(q_last(0)));
    if (err1 > 1e-5 || err2 > 1e-5) {
      ;
    } else {
      q_last(1) = temp[i];
    }
  }

  if (q_last(1) == -999) return false;
  return true;
}

/**
 * @brief 判断两个矩阵是否相等
 * @param A
 * @param B
 * @return A=B 返回true
 */
bool AnalyticalInverseKinematics::checkMatrixError(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B) {
  // 计算两个矩阵之间的误差（绝对值）
  Eigen::MatrixXd error = (A - B).cwiseAbs();

  // 计算误差中大于1e-5的元素个数
  int count = (error.array() > 1e-5).count();

  // 如果有超过一个元素的误差大于1e-5，则认为矩阵不相等
  return count < 1;
}

/**
 * @brief 判断两个向量是否相等
 * @param A
 * @param B
 * @return A=B 返回true
 */
bool AnalyticalInverseKinematics::checkVectorError(const Eigen::VectorXd &A, const Eigen::VectorXd &B) {
  if (A.size() != B.size()) return false;
  Eigen::VectorXd error = (A - B).cwiseAbs();
  // 计算误差中大于1e-5的元素个数
  int count = (error.array() > 1e-5).count();

  // 如果有超过一个元素的误差大于1e-5，则认为矩阵不相等
  return count < 1;
}

bool AnalyticalInverseKinematics::containsNaN(const Eigen::VectorXd &vec) {
  for (int i = 0; i < vec.size(); ++i) {
    if (std::isnan(vec(i))) {
      return true;  // 如果发现 NaN，返回 true
    }
  }
  return false;  // 如果没有 NaN，返回 false
}

bool AnalyticalInverseKinematics::checkResult(const Eigen::Matrix4d &T_ee, const std::vector<Eigen::VectorXd> &q_ik, std::vector<Eigen::VectorXd> &q_ik_valid,
                                              std::vector<int> &valid_idx) {
  int valid_solution_idx = 0;
  int ik_num = q_ik.size();
  if (ik_num < 1) return false;

  q_ik_valid.clear();
  valid_idx.clear();
  Eigen::Matrix4d T_ee_ik;
  for (int i = 0; i < ik_num; i++) {
    if (containsNaN(q_ik[i])) continue;
    getForwardKinematics(q_ik[i], T_ee_ik);
    if (checkMatrixError(T_ee, T_ee_ik)) {
      if (valid_solution_idx == 0) {
        valid_solution_idx = valid_solution_idx + 1;
        q_ik_valid.push_back(q_ik[i]);
        valid_idx.push_back(i);
      } else {
        std::vector<bool> isreal2(valid_solution_idx);
        for (int k = 0; k < valid_solution_idx; ++k) {
          isreal2[k] = checkVectorError(q_ik[i], q_ik_valid[k]);
        }
        int count = std::count(isreal2.begin(), isreal2.end(), true);
        if (count == 0) {
          valid_solution_idx++;
          q_ik_valid.push_back(q_ik[i]);
          valid_idx.push_back(i);
        }
      }
    }
  }

  if (valid_solution_idx < 1) return false;
  return true;
}
void AnalyticalInverseKinematics::getForwardKinematics(const Eigen::VectorXd &q, Eigen::Matrix4d &T_ee) {
  T_ee = T_0_base_;
  Eigen::Matrix4d T;
  for (int j = 0; j < q.size(); ++j) {
    getJointTransformation(q[j], mdh_d_[j], mdh_a_[j], mdh_alpha_[j], mdh_theta_[j], T);
    T_ee = T_ee * T;
  }
  T_ee = T_ee * T_7_MDH_URDF_;
}

bool AnalyticalInverseKinematics::getInverseKinematics(const Eigen::Matrix4d &T_ee, const double &q7, std::vector<Eigen::VectorXd> &q) {
  std::vector<Eigen::VectorXd> q_ik_temp(2, Eigen::VectorXd::Zero(dof_));
  std::vector<double> phi_temp(2, 0.0);
  Eigen::Matrix4d T_virtual;
  getVirtualEE(T_ee, q7, T_virtual);

  Eigen::Vector3d x_sw_0;
  Eigen::Matrix3d R_7_0;
  getSWAxisFromTransformation(T_virtual, x_sw_0, R_7_0);

  Eigen::Vector3d u_sw_0 = x_sw_0.normalized();
  Eigen::Matrix3d u_sw_0_skew;
  getSkewMatrix(u_sw_0, u_sw_0_skew);

  std::vector<Eigen::Vector4d> q_ref;
  Eigen::Matrix3d R_4_3;
  getReferenceJointPos(x_sw_0, q_ref, R_4_3);

  int idx = 0;
  Eigen::Matrix3d R_3_0_o, R_3_0, As, Bs, Cs, Aw, Bw, Cw;
  Eigen::Vector3d q_prev, q_last;
  for (int i = 0; i < 1; i++) {  // 经过验证，两组参考角，得到的臂角是差pi rad
    getReferencePlaneRotation(q_ref[i], R_3_0_o);
    getAsBsCsMatrix(u_sw_0, R_3_0_o, As, Bs, Cs);
    getAwBwCwMatrix(R_4_3, R_7_0, As, Bs, Cs, Aw, Bw, Cw);

    std::vector<double> phi;
    if (!getArmAngle(Aw, Bw, Cw, q7, phi)) continue;
    for (int j = 0; j < phi.size(); ++j) {
      // 角度修正，可能用到臂角的sin和cos值，因此以2*pi 修正，要将角度修正为-pi ~ pi
      if (phi[j] > M_PI) {
        phi[j] = -2 * M_PI + phi[j];
      }
      if (phi[j] < -M_PI) {
        phi[j] = phi[j] + 2 * M_PI;
      }
      //      std::cout << "phi[" << j << "]: " << phi[j] << std::endl;
    }
    for (int j = 0; j < phi.size(); j++) {
      if (!checkArmAngle(As, Bs, Cs, R_3_0_o, R_4_3, u_sw_0_skew, x_sw_0, phi[j], R_3_0)) continue;
      if (!getPreviousThreeJoints(R_3_0, q_prev)) continue;
      if (!getLastThreeJoints(Aw, Bw, Cw, phi[j], q_last)) continue;
      if (abs(q_last[2] - q7) > 1e-8) continue;
      q_ik_temp[idx].segment<3>(0) = q_prev;
      q_ik_temp[idx][3] = q_ref[i][3];
      q_ik_temp[idx].segment<3>(4) = q_last;
      phi_temp[idx] = phi[j];
      idx++;
    }
  }
  std::vector<int> valid_idx;
  if (!checkResult(T_ee, q_ik_temp, q, valid_idx)) return false;

  arm_angle_.clear();
  for (int i : valid_idx) {
    arm_angle_.push_back(phi_temp[i]);
  }
  return true;
}

bool AnalyticalInverseKinematics::returnArmAngle(std::vector<double> &arm_angle) {
  if (arm_angle_.size() == 0) {
    return false;
  }
  arm_angle = arm_angle_;
  return true;
}

bool AnalyticalInverseKinematics::getInverseKinematics(const Eigen::Matrix4d &T_ee, const double &q7, std::vector<Eigen::VectorXd> &q,
                                                       std::vector<double> &arm_angle) {
  if (!getInverseKinematics(T_ee, q7, q)) {
    return false;
  } 
  if (!returnArmAngle(arm_angle)) {
    return false;
  } 
  return true;
}
::std::vector<::Eigen::VectorXd> AnalyticalInverseKinematics::filterValidJointSolutions(const ::std::vector<::Eigen::VectorXd> &q_ik) {
  ::std::vector<::Eigen::VectorXd> valid_solutions;

  for (const auto &q : q_ik) {
    if (q.size() != joint_lower_.size() || q.size() != joint_upper_.size()) continue;
    // 检查是否超出范围
    if ((q.array() > joint_lower_.array()).all() && (q.array() < joint_upper_.array()).all()) {
      valid_solutions.push_back(q);
    }
  }
  return valid_solutions;
}

double AnalyticalInverseKinematics::computeMinDistanceToLimits(const Eigen::VectorXd &q) {
  Eigen::VectorXd lower_dist = q - joint_lower_;
  Eigen::VectorXd upper_dist = joint_upper_ - q;
  return lower_dist.cwiseMin(upper_dist).minCoeff();  // 取最小值
}

// 对有效解进行排序（按最小限位距离降序排列）
void AnalyticalInverseKinematics::sortSolutionsByDistance(std::vector<Eigen::VectorXd> &valid_solutions) {
  ::std::sort(valid_solutions.begin(), valid_solutions.end(),
              [&](const Eigen::VectorXd &a, const Eigen::VectorXd &b) { return computeMinDistanceToLimits(a) > computeMinDistanceToLimits(b); });
}

// 按照距离降序排列
void AnalyticalInverseKinematics::sortSolutionsByDistance(::std::vector<Eigen::VectorXd> &vec, const Eigen::VectorXd &q) {
  ::std::sort(vec.begin(), vec.end(), [&q](const Eigen::VectorXd &a, const Eigen::VectorXd &b) { return (a - q).norm() > (b - q).norm(); });
}

// 设置参考座标系与MDH基座标之间的齐次变换矩阵
void AnalyticalInverseKinematics::setT_world_mdh_base(const ::ul::math::Transform &T) {
  T_0_base_ = T * T_0_MDH_URDF_;  
  // std::cout << "T: \n" << T.matrix() << std::endl;  
  // std::cout << "T_0_base_: \n" << T_0_base_ << std::endl;   
}
}  // namespace mdl
}  // namespace ul