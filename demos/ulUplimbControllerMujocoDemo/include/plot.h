#ifndef __PLOT_CONTROLLER_H_
#define __PLOT_CONTROLLER_H_

#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>  // For gettimeofday
#include <unistd.h>
#include <zmq.h>

#include <iostream>
// #include "robot_iddp.hpp"
// #include "UplimbController.h"
#include <signal.h>
#include <ul/pack/UplimbController.h>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <mutex>
#include <queue>
#include <sstream>

#define SLEEP_DURATION 1000
#define TCP_ENDPOINT "tcp://*:9873"
// #define TCP_ENDPOINT "tcp://192.168.66.101:9873"
#define TOPIC "test1"
// test git push in CLion

void generate_data(const int& dof,
                   const double& ticks,
                   const UPLIMB_INPUT_INFO_STRUCT& stateInfo,
                   const UPLIMB_OUTPUT_INFO_STRUCT& cmdInfo,
                   const UPLIMB_VAR_OBSERVE& varInfo,
                   const ::std::vector<Eigen::VectorXd>& act_ee_vel,
                   const ::std::vector<Eigen::VectorXd>& act_ee_pose,
                   const ::std::vector<Eigen::VectorXd>& tau_g,
                   const ::std::vector<Eigen::VectorXd>& tau_nle,
                   ::std::string& output) {
  ::std::ostringstream oss;
  oss << ::std::fixed << ::std::setprecision(10);

  // 开始 JSON
  oss << "{"
      << "\"ticks\": " << ticks << ", "
      << "\"timestamp\": " << stateInfo.upTime / 1e9 << ", "
      << "\"stateInfo\": {"
      << "\"mode\": [";

  // 遍历 mode 数组
  for (size_t i = 0; i < dof; ++i) { oss << static_cast<int>(stateInfo.mode[i]) << (i < dof - 1 ? "," : ""); }

  oss << "], \"statuswd\": [";
  for (size_t i = 0; i < dof; ++i) { oss << stateInfo.statuswd[i] << (i < dof - 1 ? "," : ""); }

  oss << "], \"act_pos\": [";
  for (size_t i = 0; i < dof; ++i) { oss << stateInfo.act_pos[i] << (i < dof - 1 ? "," : ""); }

  oss << "], \"act_vel\": [";
  for (size_t i = 0; i < dof; ++i) { oss << stateInfo.act_vel[i] << (i < dof - 1 ? "," : ""); }

  oss << "], \"act_acc\": [";
  for (size_t i = 0; i < dof; ++i) { oss << stateInfo.act_acc[i] << (i < dof - 1 ? "," : ""); }

  oss << "], \"act_tau\": [";
  for (size_t i = 0; i < dof; ++i) { oss << stateInfo.act_tau[i] << (i < dof - 1 ? "," : ""); }

  oss << "], \"act_ee_vel\": [";
  // 适配不同自由度
  for (size_t i = 0; i < act_ee_vel.size(); ++i) {
    for (size_t j = 0; j < act_ee_vel[i].size(); ++j) {
      oss << act_ee_vel[i][j] << ((i < act_ee_vel.size() - 1 || j < act_ee_vel[i].size() - 1) ? "," : "");
    }
  }

  oss << "], \"act_ee_pose\": [";
  for (size_t i = 0; i < act_ee_pose.size(); ++i) {
    for (size_t j = 0; j < act_ee_pose[i].size(); ++j) {
      oss << act_ee_pose[i][j] << ((i < act_ee_pose.size() - 1 || j < act_ee_pose[i].size() - 1) ? "," : "");
    }
  }

  oss << "], \"tau_nle\": [";
  for (size_t i = 0; i < tau_nle.size(); ++i) {
    for (size_t j = 0; j < tau_nle[i].size(); ++j) {
      oss << tau_nle[i][j] << ((i < tau_nle.size() - 1 || j < tau_nle[i].size() - 1) ? "," : "");
    }
  }

  oss << "], \"tau_g\": [";
  for (size_t i = 0; i < tau_g.size(); ++i) {
    for (size_t j = 0; j < tau_g[i].size(); ++j) {
      oss << tau_g[i][j] << ((i < tau_g.size() - 1 || j < tau_g[i].size() - 1) ? "," : "");
    }
  }

  // 关闭 stateInfo，打开 varInfo
  oss << "]}, \"varInfo\": {"
      << "\"var1\": [";
  for (size_t i = 0; i < dof; ++i) { oss << varInfo.var1[i] << (i < dof - 1 ? "," : ""); }

  oss << "], \"var6\": [";
  for (size_t i = 0; i < dof; ++i) { oss << varInfo.var6[i] << (i < dof - 1 ? "," : ""); }

  oss << "], \"var7\": [";
  for (size_t i = 0; i < dof; ++i) { oss << varInfo.var7[i] << (i < dof - 1 ? "," : ""); }

  oss << "], \"var8\": [";
  for (size_t i = 0; i < dof; ++i) { oss << varInfo.var8[i] << (i < dof - 1 ? "," : ""); }

  oss << "], \"var9\": [";
  for (size_t i = 0; i < dof; ++i) { oss << varInfo.var9[i] << (i < dof - 1 ? "," : ""); }

  oss << "], \"var10\": [";
  for (size_t i = 0; i < dof; ++i) { oss << varInfo.var10[i] << (i < dof - 1 ? "," : ""); }

  // 关闭 varInfo，打开 cmdInfo
  oss << "]}, \"cmdInfo\": {"
      << "\"offset_torque\": [";
  for (size_t i = 0; i < dof; ++i) { oss << cmdInfo.offset_torque[i] << (i < dof - 1 ? "," : ""); }

  oss << "], \"offset_vel\": [";
  for (size_t i = 0; i < dof; ++i) { oss << cmdInfo.offset_vel[i] << (i < dof - 1 ? "," : ""); }

  oss << "], \"mode_operation\": [";
  for (size_t i = 0; i < dof; ++i) { oss << static_cast<int>(cmdInfo.mode_operation[i]) << (i < dof - 1 ? "," : ""); }

  oss << "], \"ctrlwd\": [";
  for (size_t i = 0; i < dof; ++i) { oss << cmdInfo.ctrlwd[i] << (i < dof - 1 ? "," : ""); }

  oss << "], \"cmd_pos\": [";
  for (size_t i = 0; i < dof; ++i) { oss << cmdInfo.cmd_pos[i] << (i < dof - 1 ? "," : ""); }

  oss << "], \"cmd_vel\": [";
  for (size_t i = 0; i < dof; ++i) { oss << cmdInfo.cmd_vel[i] << (i < dof - 1 ? "," : ""); }

  oss << "], \"cmd_tau\": [";
  for (size_t i = 0; i < dof; ++i) { oss << cmdInfo.cmd_tau[i] << (i < dof - 1 ? "," : ""); }

  // 关闭整个 JSON
  oss << "]}}";

  output = oss.str();
}

void* zmq_publisher(::std::atomic<bool>* keepRunning, int* dof) {
  ::std::string input;

  UPLIMB_INPUT_INFO_STRUCT stateInfo;
  UPLIMB_OUTPUT_INFO_STRUCT cmdInfo;
  UPLIMB_VAR_OBSERVE varInfo;

  ::std::vector<Eigen::VectorXd> act_ee_vel(2, Eigen::VectorXd(6));
  ::std::vector<Eigen::VectorXd> act_ee_pose(2, Eigen::VectorXd(6));
  ::std::vector<Eigen::VectorXd> tau_g(2, Eigen::VectorXd(7));
  ::std::vector<Eigen::VectorXd> tau_nle(2, Eigen::VectorXd(7));

  unsigned long long tp_up;
  double ticks = 0;

  void* context = zmq_ctx_new();
  void* publisher = zmq_socket(context, ZMQ_PUB);
  int rc = zmq_bind(publisher, TCP_ENDPOINT);
  if (rc != 0) {
    perror("zmq_bind");
    return NULL;
  }

  while (keepRunning->load(::std::memory_order_relaxed)) {
    ::std::string json_output;

    // 获取当前状态
    getArmState(&stateInfo);
    getArmCMD(&cmdInfo);
    getArmVar(&varInfo);

    // 获取运动数据
    //    act_ee_vel[0] = getEEVel(0); // 左臂
    //    act_ee_vel[1] = getEEVel(1); // 右臂

    //    act_ee_pose[0] = getForwardKinematics(0); // 左臂
    //    act_ee_pose[1] = getForwardKinematics(1); // 右臂

    //    getGravityTorque(tau_g[0], 0);  // 左臂
    //    getGravityTorque(tau_g[1], 1);  // 右臂
    //
    //    getNLETorque(tau_nle[0], 0);  // 左臂
    //    getNLETorque(tau_nle[1], 1);  // 右臂

    // 调用新的 generate_data 生成 JSON
    generate_data(*dof, ticks, stateInfo, cmdInfo, varInfo, act_ee_vel, act_ee_pose, tau_g, tau_nle, json_output);

//    // **检查 JSON 是否正确**
//    if (json_output.empty()) {
//      ::std::cerr << "Error: Generated JSON is empty!" << ::std::endl;
//      continue;  // 跳过发送，避免无效数据
//    }
//    ::std::cout << "Generated JSON: " << json_output << ::std::endl;

    // 发送 JSON 数据
    zmq_send(publisher, TOPIC, strlen(TOPIC), ZMQ_SNDMORE);
    zmq_send(publisher, json_output.c_str(), json_output.size(), 0);

    // 更新时间戳
    tp_up = stateInfo.upTime;
    ticks += 0.001;

    usleep(SLEEP_DURATION);
  }

  zmq_close(publisher);
  zmq_ctx_destroy(context);
  printf("===== Plot Task finished!\n");
  return NULL;
}

void set_thread_priority(pthread_t thread, int policy, int priority) {
  struct sched_param sch_params;
  sch_params.sched_priority = priority;

  if (pthread_setschedparam(thread, policy, &sch_params)) {
    fprintf(stderr, "Failed to set thread scheduling: %s\n", strerror(errno));
  }
}

::std::vector<Eigen::Vector3d> quaternionsToRPY(const ::std::vector<Eigen::Quaterniond>& quaternions) {
  ::std::vector<Eigen::Vector3d> rpyAngles;
  rpyAngles.reserve(quaternions.size());

  for (const auto& quat : quaternions) {
    // 将四元数转换为旋转矩阵
    Eigen::Matrix3d rotationMatrix = quat.toRotationMatrix();

    // ::std::cout << "pose_T: " << rotationMatrix << ::std::endl;

    // 从旋转矩阵提取RPY角
    Eigen::Vector3d rpy = rotationMatrix.eulerAngles(2, 1, 0);  // ZYX顺序
    rpyAngles.push_back(rpy);
  }

  return rpyAngles;
}

#endif