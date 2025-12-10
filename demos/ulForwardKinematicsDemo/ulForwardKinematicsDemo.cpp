/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 24-01-09
 * @Version       : 0.0.1
 * @File          : ulForwardKinematicsDemo.cpp
 ******************************************************************************
 */
#include <iostream>
#include <ul/math/Vector.h>
//#include <ul/mdl/Revolute.h>
#include <ul/mdl/Kinematic.h>

int main() {
  try {
    const char* user = getenv("USER");
    ::std::cout << "Hello, " << user << "!" << ::std::endl;
    // 定义基础路径
    ::std::string basePath = "/home/";
    ::std::string urdf_file = basePath + user + "/Documents/example_code/robot_models/URDF-H1_Pro/urdf/URDF-A1_Pro_no_collision.urdf";
    ::ul::mdl::Kinematic robot(urdf_file);

    // 获取自由度
    ::std::size_t dof = robot.getDof();
    ::std::cout << "Robot DOF: " << dof << ::std::endl;

    // 设置机器人位置
    ::ul::math::Vector q(dof);  // 确保 q 的大小与 DOF 匹配
    robot.setPosition(q);

    ::std::cout << "Position set successfully." << ::std::endl;

    ::ul::math::Transform op_trans = robot.getOperationalTransform("TCP_R");
    ::std::cout << "Operational Transform: \n" << op_trans.matrix() << ::std::endl;

    robot.calculateJacobian(1);

    ::ul::math::Matrix J_left;
    J_left = robot.getJacobian();
    ::std::cout << "Jacobian Matrix Size: " << J_left.rows() << "x" << J_left.cols() << ::std::endl;
    ::std::cout << "Jacobian Matrix: \n" << J_left << ::std::endl;


    op_trans = robot.getOperationalTransform("TCP_L");
    ::std::cout << "Operational Transform: \n" << op_trans.matrix() << ::std::endl;

    robot.calculateJacobian(1);

    ::ul::math::Matrix J_right = robot.getJacobian();
    ::std::cout << "Jacobian Matrix Size: " << J_right.rows() << "x" << J_right.cols() << ::std::endl;
    ::std::cout << "Jacobian Matrix: \n" << J_right << ::std::endl;

  } catch (const ::std::exception& e) {
    ::std::cerr << "Error: " << e.what() << ::std::endl;
  }

  return 0;
}