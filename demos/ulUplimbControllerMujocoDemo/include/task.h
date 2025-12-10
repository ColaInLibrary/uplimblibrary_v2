/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-6-10
 * @Version       : 0.0.1
 * @File          : task.h
 ******************************************************************************
 */

#ifndef UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_TASK_H_
#define UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_TASK_H_


#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <unistd.h>
#include <pwd.h>
#include <ul/pack/UplimbController.h>

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "plot.h"
#include <unordered_map>

void executeCommand0() {
  auto ee_pose = getForwardKinematics();
  ::std::cout << "left ee pose: " << ee_pose[0].transpose() << ::std::endl;
  ::std::cout << "right ee pose: " << ee_pose[1].transpose() << ::std::endl;

  ::Eigen::VectorXd target_q(getDof());
  target_q.setZero();
  ::std::cout << getJointActualPositions() << ::std::endl;
  if (!moveJ(target_q, 3, 1.4, false)) return;
}

void executeCommand1() {
  ::Eigen::VectorXd target_q(getDof());
  //  target_q << -0.049, -0.508, -0.082, -1.302,  0.809, -0.01, -0.15,
  target_q << -0.035,  0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
      -0.223363, -0.38122, 0.189163, -0.819702, 0.566464, -0.0839337, 0.0170038,
      0, 0, 0;
  //  target_q.setZero();
  //  target_q[0] = -0.049;
  moveJ(target_q, 3, 1.4, false);
}

void executeCommand2() {
  ::std::vector<::std::vector<double>> path(3, std::vector<double>(getDof(), 0));
  ::Eigen::VectorXd target_q(getDof());
  target_q <<
      -0.035,  0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
      -0.223363, -0.38122, 0.189163, -0.819702, 0.566464, -0.0839337, 0.0170038,
      0, 0, 0;
  for (int i = 0; i < getDof(); ++i) {
    path[0][i] = target_q[i];
  }
  target_q << -0.035, 0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
      -0.049, -0.508, -0.082, -1.302, 0.809, -0.01, -0.15,
      0, 0, 0;
  for (int i = 0; i < getDof(); ++i) {
    path[1][i] = target_q[i];
  }
  target_q <<
      -0.035,  0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
      -0.223363, -0.38122, 0.189163, -0.819702, 0.566464, -0.0839337, 0.0170038,
      0, 0, 0;
  for (int i = 0; i < getDof(); ++i) {
    path[2][i] = target_q[i];
  }
  moveJ(path, 2, false, 0x0F);
}

void executeCommand3() {
  ::Eigen::VectorXd target_qd(getDof());
  target_qd.setZero();
  target_qd[1] = 0.3;
  speedJ(target_qd, 0.5, 2);
}

void executeCommand4() {
  speedStop();
}

void executeCommand5() {
  for (int cnt = 0; cnt < 1; ++cnt) {  // 循环执行多少次
    ::std::cout << ">>>>>> cnt = " << cnt << ::std::endl;
    ::Eigen::VectorXd target_q(getDof());
    target_q <<
        -0.035,  0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
        -0.049, -0.508, -0.082, -1.302,  0.809, -0.01, -0.15,
        0, 0, 0;
    moveJ(target_q, 2, 5.4, false);
    ::std::vector<::Eigen::Matrix<double, 6, 1>> ee_pose(2);
    ee_pose = getForwardKinematics();
    ::std::cout << "left ee pose: " << ee_pose[0].transpose() << ::std::endl;
    ::std::cout << "right ee pose: " << ee_pose[1].transpose() << ::std::endl;
    ::std::vector<::Eigen::Matrix<double, 6, 1>> target_ee_pose(2);
    for (int i = 0; i < 2; ++i) {
      target_ee_pose[i] = ee_pose[i];
      target_ee_pose[i][2] += 0.05;
      target_ee_pose[i][3] += 0.3;
    }
    moveL(target_ee_pose, 0.1, 1, false);
    ee_pose = getForwardKinematics();
    ::std::cout << "left ee pose: " << ee_pose[0].transpose() << ::std::endl;
    ::std::cout << "right ee pose: " << ee_pose[1].transpose() << ::std::endl;
  }
}

// 阻抗控制测试
void executeCommand6() {
  // 先用moveJ运动指定位置
  ::Eigen::VectorXd target_q(getDof());
  target_q <<
      -0.035,  0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
      -M_PI_2, -0.2, M_PI, -1.57,  0.809, -0.01, -0.15,
      0, 0, 0;
  moveJ(target_q, 2, 5.4, false);
}



#endif  // UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_TASK_H_
