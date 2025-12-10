/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-02-08
 * @Version       : 0.0.1
 * @File          : ulQuaternionDemo.cpp
 ******************************************************************************
 */
#include <iostream>
#include <ul/math/Vector.h>
#include <ul/math/Quaternion.h>
// test
int main() {
  // 定义两个四元数
  ::ul::math::Quaternion q1 = ::ul::math::Quaternion(::Eigen::AngleAxisd(M_PI / 4, ::ul::math::Vector3::UnitX())); // 绕 X 轴旋转 45 度
  ::ul::math::Quaternion q2 = ::ul::math::Quaternion(::Eigen::AngleAxisd(M_PI / 2, ::ul::math::Vector3::UnitY())); // 绕 Y 轴旋转 90 度

  double t = 0.5; // 插值参数 (0 <= t <= 1)

  // 计算球面线性插值 (SLERP)
  ::ul::math::Quaternion slerp_result = q1.slerp(t, q2);
  ::std::cout << "q1: " << q1.vec().transpose() << ::std::endl;
  ::std::cout << "q2: " << q2.vec().transpose() << ::std::endl;
  ::std::cout << "SLERP Result: " << slerp_result.coeffs().transpose() << ::std::endl;
  ::std::cout << "angular velocity: " << q1.angularVelocity(slerp_result, 0.01).transpose() << ::std::endl;

  // 计算 SLERP 的一阶导数
  ::ul::math::Quaternion slerp_first_derivative = q1.slerp(t, q2).slerpFirstDerivative(t, q2);
  ::std::cout << "SLERP First Derivative: " << slerp_first_derivative.coeffs().transpose() << ::std::endl;

  // 计算 SLERP 的二阶导数
  ::ul::math::Quaternion slerp_second_derivative = q1.slerp(t, q2).slerpSecondDerivative(t, q2);
  ::std::cout << "SLERP Second Derivative: " << slerp_second_derivative.coeffs().transpose() << ::std::endl;

  // 定义一个四元数（旋转 90 度绕 Z 轴）
  ::ul::math::Quaternion q = ::ul::math::Quaternion(::Eigen::AngleAxisd(M_PI / 2, ::ul::math::Vector3::UnitZ()));

  // 计算四元数的对数 log(q)
  ::ul::math::Quaternion q_log = q.log();
  ::std::cout << "log(q): " << q_log.coeffs().transpose() << ::std::endl;

  // 计算四元数的指数 exp(log(q))，应当恢复原四元数
  ::ul::math::Quaternion q_exp = q_log.exp();
  ::std::cout << "exp(log(q)): " << q_exp.coeffs().transpose() << ::std::endl;

  return 0;
}
