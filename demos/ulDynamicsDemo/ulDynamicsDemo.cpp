/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 24-01-09
 * @Version       : 0.0.1
 * @File          : ulDynamicsDemo.cpp
 ******************************************************************************
 */
#include <iostream>
#include <ul/math/Vector.h>
#include <ul/mdl/Revolute.h>
#include <ul/mdl/Kinematic.h>
#include <ul/mdl/Dynamic.h>

int main() {
  try {
    ::std::string urdf_file = "/home/ah/Documents/example_code/robot_models/NavigatorDualArmV2/urdf/newarmurdf0718.urdf";
    ::ul::mdl::Dynamic robot(urdf_file);

    // 获取自由度
    ::std::size_t dof = robot.getDof();
    ::std::cout << "Robot DOF: " << dof << ::std::endl;

    // 设置机器人位置
    ::ul::math::Vector q(dof);  // 确保 q 的大小与 DOF 匹配
    robot.setPosition(q);

    // 计算重力矩
    robot.calculateGravity();
    ::ul::math::Vector G = robot.getGravity();
    ::std::cout << "Gravity: " << G.transpose() << ::std::endl;

  } catch (const ::std::exception& e) {
    ::std::cerr << "Error: " << e.what() << ::std::endl;
  }

  return 0;
}