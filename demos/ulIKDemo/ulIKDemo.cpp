/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 24-02-21
 * @Version       : 0.0.1
 * @File          : ulIKDemo.cpp
 ******************************************************************************
 */
#include <iostream>
#include <random>
#include <ul/math/Vector.h>
#include <ul/math/Transform.h>
#include <ul/mdl/AnalyticalInverseKinematics.h>
#include <ul/math/Pose2Transformation.h>

int main() {
  // demo case 1
  ::ul::mdl::AnalyticalInverseKinematics ik;
  std::string arm_type = "RIGHT";
  ik.init(arm_type);
  std::cout << "ik init succeed!" << std::endl;
  Eigen::Matrix4d T_ee;
  double q7 = -3.14;
  T_ee << 0, 0, 1, 0.289583,
      -1, 0, 0, -0.355,
      0, -1, 0, 0.269931,
      0, 0, 0, 1;
  Eigen::VectorXd q(7);
  // 定义随机数生成器和分布
  std::default_random_engine generator;
  generator.seed(std::random_device{}()); // 使用随机设备种子初始化以增加随机性
  // 定义每个维度的最小值和最大值
  Eigen::VectorXd minVals(7);
  Eigen::VectorXd maxVals(7);

  // 为minVals和maxVals赋值（这里只是示例值，你应该根据需要设置）
  if (arm_type == "LEFT") {
    minVals << -3.0,  0.0, -2.9, -2.0, -2.9, -1.5, -1.5;
    maxVals <<  1.0, -3.0,  2.9,  0.0,  2.9,  1.5,  1.5;
  } else if (arm_type == "RIGHT") {
    minVals << -3.0, -3.0, -2.9, -2.0, -2.9, -1.5, -1.5;
    maxVals <<  1.0,  0.0,  2.9,  0.0,  2.9,  1.5,  1.5;
  }

  std::vector<Eigen::VectorXd> q_ik, q_ik_valid;
  std::vector<double> arm_angle;
  int cnt = 0;
  int success_cnt = 0;
  int fail_cnt = 0;
  while (cnt < 10000) {
    // 用随机数填充向量
    for (int i = 0; i < q.size(); ++i) {
      std::uniform_real_distribution<double> distribution(minVals(i), maxVals(i));
      q(i) = distribution(generator);
    }
    ik.getForwardKinematics(q, T_ee);
    if(ik.getInverseKinematics(T_ee, q(6), q_ik, arm_angle)) {
      success_cnt++;
//      std::cout << "\n------------------------------\ncnt: " << cnt << std::endl;
//      std::cout << "q: " << q.transpose() << std::endl;
//      std::cout << "q7 = " << q(6) << ", q_ik size = " << q_ik.size() << std::endl;
      for (int i = 0; i < q_ik.size(); ++i) {
//        std::cout << "arm_angle slolution " << i << ": " << arm_angle[i] << std::endl;
//        std::cout << "q_ik slolution " << i << ": " << q_ik[i].transpose() << std::endl;
        ik.getForwardKinematics(q_ik[i], T_ee);
//        std::cout << "T_ee: \n" << T_ee << std::endl;
      }
    } else {
      ik.getInverseKinematics(T_ee, q(6), q_ik, arm_angle);
      fail_cnt++;
      std::cout << "fail cnt: " << fail_cnt << std::endl;
      std::cout << "q: " << q.transpose() << std::endl;
      std::cout << "T_ee: \n" << T_ee << std::endl;
    }
    cnt++;
  }
  std::cout << "success_cnt: " << success_cnt << ", fail_cnt: " << fail_cnt << std::endl;

  // 右臂关节角求解
  ::ul::math::Vector pose(6);
  pose << 0.46906487, -0.0305704,   0.29026702,  1.15489736, -1.15567176, -0.72661238;
  ::ul::math::Transform T_ee_pose;
  ::ul::math::Pose2Transformation(pose, T_ee_pose);

  q7 = -1.57;
  while (q7 < 1.57) {
    if(ik.getInverseKinematics(T_ee_pose.matrix(), q7, q_ik, arm_angle)) {
      for (int i = 0; i < q_ik.size(); ++i) {
        ::std::cout << "q_ik = " << q_ik[i].transpose() << ", phi = " << arm_angle[i] << std::endl;
      }
      q_ik_valid = ik.filterValidJointSolutions(q_ik);
      for (int i = 0; i < q_ik_valid.size(); ++i) {
        ::std::cout << "q_ik_valid = " << q_ik_valid[i].transpose() << "" << std::endl;
      }
    }
    q7 += 0.01;
  }

//  q << 0, -1, -M_PI_2, -1, 0, 0, 0;
  q << -0.21594406, -0.28182024, -0.04215794, -0.80653754,  0.78982856, -0.15639932, 0.0864175;
//  q7 = 0.0864175;
  q7 = 1.12426;
  ik.getForwardKinematics(q, T_ee);
  ::std::cout << "T_ee: \n" << T_ee << std::endl;
  if(ik.getInverseKinematics(T_ee.matrix(), q7, q_ik, arm_angle)) {
    for (int i = 0; i < q_ik.size(); ++i) {
      ::std::cout << "q_ik = " << q_ik[i].transpose() << ", phi = " << arm_angle[i] << std::endl;
    }
  }
  ::std::cout << "----------";
  return 1;
}