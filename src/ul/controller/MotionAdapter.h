/**
 ******************************************************************************
 * @Description   : 多机型适配
 * @author        : AN Hao
 * @Date          : 25-5-21
 * @Version       : 0.0.1
 * @File          : MotionAdapter.h
 ******************************************************************************
 */

#ifndef UL_SRC_UL_PACK_MOTIONADAPTER_H_
#define UL_SRC_UL_PACK_MOTIONADAPTER_H_

#include "MotionStrategy.h"

class Robot180Adapter : public MotionStrategy {
 public:
  Robot180Adapter() {
    q_home.resize(4);
    q_home[0].resize(ARM_JOINT_NUM);
    q_home[0] << -0.035, 0.533, -0.096, -1.348, -0.787, 0.008, 0.200;

    q_home[1].resize(R180_ARM_JOINT_NUM);
    q_home[1] << -0.035, -0.533, -0.096;

    q_home[2].resize(HEAD_JOINT_NUM);
    q_home[2] << 0.0, 0.0;

    q_home[3].resize(R180_WAIST_JOINT_NUM);
    q_home[3] << 0.0, 0.0;

    joint_num = {ARM_JOINT_NUM, R180_ARM_JOINT_NUM, HEAD_JOINT_NUM, R180_WAIST_JOINT_NUM};

    cartesian_config.cartesian_select = {{0, 1, 2, 3, 4, 5}, {0, 1, 2}};  // 右臂控位姿
    cartesian_config.joint_name_first = {"Shoulder_Y_L", "Shoulder_Y_R"};
    cartesian_config.ee_name = {"Tcp_L", "Tcp_R"};
  };
  virtual ~Robot180Adapter() = default;
};

class A3DualArmAdapter : public MotionStrategy {
 public:
  A3DualArmAdapter() {
    q_home.resize(2);
    q_home[0].resize(A3_ARM_JOINT_NUM);
    q_home[0] << 0, -0.035, 0.533, -0.096, -1.348, -0.787, 0.008, 0.200;

    q_home[1].resize(A3_ARM_JOINT_NUM);
    q_home[1] << 0, -0.035, -0.533, -0.096, -1.348, 0.787, -0.008, -0.200;

    joint_num = {A3_ARM_JOINT_NUM, A3_ARM_JOINT_NUM};

    cartesian_config.cartesian_select = {{0, 1, 2, 3, 4, 5}, {0, 1, 2, 3, 4, 5}};
    cartesian_config.joint_name_first = {"Torso-Z-L", "Torso-Z-R"};
    cartesian_config.ee_name = {"Tcp_L", "Tcp_R"};
  };
  virtual ~A3DualArmAdapter() = default;
};

class WA3Adapter : public MotionStrategy {
 public:
  WA3Adapter() {
    q_home.resize(4);
    q_home[0].resize(A3_ARM_JOINT_NUM);
    q_home[0] << 0, -0.035, 0.533, -0.096, -1.348, -0.787, 0.008, 0.200;

    q_home[1].resize(A3_ARM_JOINT_NUM);
    q_home[1] << 0, -0.035, -0.533, -0.096, -1.348, 0.787, -0.008, -0.200;

    q_home[2].resize(HEAD_JOINT_NUM);
    q_home[2] << 0.0, 0.0;

    q_home[3].resize(WA3_WAIST_JOINT_NUM);
    q_home[3] << 0.0, 0.0, 0.0, 0.0;

    joint_num = {A3_ARM_JOINT_NUM, A3_ARM_JOINT_NUM, HEAD_JOINT_NUM, WA3_WAIST_JOINT_NUM};

    cartesian_config.cartesian_select = {{0, 1, 2, 3, 4, 5}, {0, 1, 2, 3, 4, 5}};
    cartesian_config.joint_name_first = {"Shoulder_Z_L", "Shoulder_Z_R"};
    cartesian_config.ee_name = {"Tcp_L", "Tcp_R"};
  };
  virtual ~WA3Adapter() = default;
};

class WA2Adapter : public MotionStrategy {
 public:
  WA2Adapter() {
    q_home.resize(4);
    q_home[0].resize(A2_ARM_JOINT_NUM);
    q_home[0] << 0, -0.035, 0.533, -0.096, -1.348, -0.787, 0.008, 0.200;

    q_home[1].resize(A2_ARM_JOINT_NUM);
    q_home[1] << 0, -0.035, -0.533, -0.096, -1.348, 0.787, -0.008, -0.200;

    q_home[2].resize(HEAD_JOINT_NUM);
    q_home[2] << 0.0, 0.0;

    q_home[3].resize(WA2_WAIST_JOINT_NUM);
    q_home[3] << 0.0, 0.0, 0.0, 0.0;

    joint_num = {A2_ARM_JOINT_NUM, A2_ARM_JOINT_NUM, HEAD_JOINT_NUM, WA2_WAIST_JOINT_NUM};

    cartesian_config.cartesian_select = {{0, 1, 2, 3, 4, 5}, {0, 1, 2, 3, 4, 5}};
    cartesian_config.joint_name_first = {"Shoulder_Z_L", "Shoulder_Z_R"};
    cartesian_config.ee_name = {"Tcp_L", "Tcp_R"};
  };
  virtual ~WA2Adapter() = default;
};

class DualArmAdapter : public MotionStrategy {
 public:
  DualArmAdapter() {
    q_home.resize(3);
    q_home[0].resize(ARM_JOINT_NUM);
    q_home[0] << -0.035, 0.533, -0.096, -1.348, -0.787, 0.008, 0.200;

    q_home[1].resize(ARM_JOINT_NUM);
    q_home[1] << -0.035, -0.533, -0.096, -1.348, 0.787, -0.008, -0.200;

    q_home[2].resize(HEAD_JOINT_NUM);
    q_home[2] << 0.0, 0.0;

    joint_num = {ARM_JOINT_NUM, ARM_JOINT_NUM, HEAD_JOINT_NUM};

    cartesian_config.cartesian_select = {{0, 1, 2, 3, 4, 5}, {0, 1, 2, 3, 4, 5}};
    cartesian_config.joint_name_first = {"Shoulder_Y_L", "Shoulder_Y_R"};
    cartesian_config.ee_name = {"Tcp_L", "Tcp_R"};
  };
  virtual ~DualArmAdapter() = default;
};

class UpperBodyAdapter : public MotionStrategy {
 public:
  UpperBodyAdapter() {
    q_home.resize(4);
    q_home[0].resize(ARM_JOINT_NUM);
    q_home[0] << -0.035, 0.533, -0.096, -1.348, -0.787, 0.008, 0.200;

    q_home[1].resize(ARM_JOINT_NUM);
    q_home[1] << -0.035, -0.533, -0.096, -1.348, 0.787, -0.008, -0.200;

    q_home[2].resize(HEAD_JOINT_NUM);
    q_home[2] << 0.0, 0.0;

    q_home[3].resize(WAIST_JOINT_NUM);
    q_home[3] << 0.0;

    joint_num = {ARM_JOINT_NUM, ARM_JOINT_NUM, HEAD_JOINT_NUM, WAIST_JOINT_NUM};

    cartesian_config.cartesian_select = {{0, 1, 2, 3, 4, 5}, {0, 1, 2, 3, 4, 5}};
    cartesian_config.joint_name_first = {"Shoulder_Y_L", "Shoulder_Y_R"};
    cartesian_config.ee_name = {"Tcp_L", "Tcp_R"};
  };
  virtual ~UpperBodyAdapter() = default;
};

class WheelArmAdapter : public MotionStrategy {
 public:
  WheelArmAdapter() {
    q_home.resize(5);
    q_home[0].resize(ARM_JOINT_NUM);
    q_home[0] << -0.25, 0.75, 0.38, -1.15, 0.1, -0.12, 0.1;

    q_home[1].resize(ARM_JOINT_NUM);
    q_home[1] << -0.25, -0.75, -0.38, -1.15, -0.1, -0.12, 0.1;

    q_home[2].resize(HEAD_JOINT_NUM);
    q_home[2] << 0.0, 0.0;

    q_home[3].resize(WHEEL_WAIST_LIFT_JOINT_NUM);
    q_home[3] << 0.01;

    q_home[4].resize(WHEEL_WAIST_JOINT_NUM);
    q_home[4] << 0.0, 0.0;

    joint_num = {ARM_JOINT_NUM, ARM_JOINT_NUM, HEAD_JOINT_NUM, WHEEL_WAIST_LIFT_JOINT_NUM, WHEEL_WAIST_JOINT_NUM};

    cartesian_config.cartesian_select = {{0, 1, 2, 3, 4, 5}, {0, 1, 2, 3, 4, 5}};
    cartesian_config.joint_name_first = {"Shoulder_Y_L", "Shoulder_Y_R"};
    cartesian_config.ee_name = {"Tcp_L", "Tcp_R"};
  };
  virtual ~WheelArmAdapter() = default;
};

class I3DualArmAdapter : public MotionStrategy {
 public:
  I3DualArmAdapter() {
    q_home.resize(3);
    q_home[0].resize(I3_ARM_JOINT_NUM);
    q_home[0] << -0.25, 0.75, 0.38, -1.15;

    q_home[1].resize(I3_ARM_JOINT_NUM);
    q_home[1] << -0.25, -0.75, -0.38, -1.15;

    q_home[2].resize(HEAD_JOINT_NUM);
    q_home[2] << 0.0, 0.0;

    joint_num = {I3_ARM_JOINT_NUM, I3_ARM_JOINT_NUM, HEAD_JOINT_NUM};

    cartesian_config.cartesian_select = {{0, 1, 2, 3}, {0, 1, 2, 3}};
    cartesian_config.joint_name_first = {"Shoulder_Y_L", "Shoulder_Y_R"};
    // cartesian_config.ee_name = {"Hand_L", "Hand_R"};
    cartesian_config.ee_name = {"Elbow_L", "Elbow_R"};
  };
  virtual ~I3DualArmAdapter() = default;
};
#endif  // UL_SRC_UL_PACK_MOTIONADAPTER_H_
