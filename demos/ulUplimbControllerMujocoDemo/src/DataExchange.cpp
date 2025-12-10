/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-11-12
 * @Version       : 0.0.1
 * @File          : DataExchange.cpp
 ******************************************************************************
 */

#include "DataExchange.h"
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <ul/pack/UplimbController.h>

// 全局变量声明（与原始代码保持一致）
extern UPLIMB_INPUT_INFO_STRUCT measured_info;
extern UPLIMB_OUTPUT_INFO_STRUCT command_info;
extern UPLIMB_VAR_OBSERVE var_info;
//extern int dof;
//extern unsigned long long int ticks;


DataExchange::DataExchange() {
  // 构造函数
}

void DataExchange::initialize(int dof) {
  this->dof = dof;
  std::cout << "DataExchange initialized with DOF: " << this->dof << std::endl;
}

void DataExchange::exchangeData(const mjModel* m, mjData* d, bool isPositionControl) {
  // 获取六维力信息（与原始代码完全一致）
  std::vector<const char*> force_sensor_names = {"TCP_L_force_sensor", "TCP_R_force_sensor"};
  std::vector<const char*> torque_sensor_names = {"TCP_R_torque_sensor", "TCP_R_torque_sensor"};
  std::vector<Eigen::VectorXd> ft_wrench(2, Eigen::VectorXd::Zero(6));

  for (int i = 0; i < force_sensor_names.size(); ++i) {
    int force_sensor_id = mj_name2id(m, mjOBJ_SENSOR, force_sensor_names[i]);
    int torque_sensor_id = mj_name2id(m, mjOBJ_SENSOR, torque_sensor_names[i]);
    mjtNum* force = d->sensordata + m->sensor_adr[force_sensor_id];
    mjtNum* torque = d->sensordata + m->sensor_adr[torque_sensor_id];
    for (int j = 0; j < 3; ++j) {
      // 给右手力传感器赋值
      ft_wrench[i][j] = force[j];
      ft_wrench[i][j + 3] = torque[j];
    }
  }

  // 更新仿真时间（与原始代码完全一致）
  ticks++;
  measured_info.upTime = ticks;

  // 获取腰部仿真数据（与原始代码完全一致）
  measured_info.act_pos[dof - 1] = d->qpos[0];
  measured_info.act_vel[dof - 1] = d->qvel[0];
  measured_info.act_acc[dof - 1] = d->qacc[0];

  // 获取颈部仿真数据（与原始代码完全一致）
  measured_info.act_pos[dof - 2] = d->qpos[dof - 1];
  measured_info.act_pos[dof - 3] = d->qpos[dof - 2];
  measured_info.act_vel[dof - 2] = d->qvel[dof - 1];
  measured_info.act_vel[dof - 3] = d->qvel[dof - 2];
  measured_info.act_acc[dof - 2] = d->qacc[dof - 1];
  measured_info.act_acc[dof - 3] = d->qacc[dof - 2];

  // 获取左臂、右臂仿真数据（与原始代码完全一致）
  for (int i = 0; i < 7; ++i) {
    measured_info.act_pos[i] = d->qpos[i + 7 + 1];
    measured_info.act_vel[i] = d->qvel[i + 7 + 1];
    measured_info.act_acc[i] = d->qacc[i + 7 + 1];
    measured_info.act_pos[i + 7] = d->qpos[i + 1];
    measured_info.act_vel[i + 7] = d->qvel[i + 1];
    measured_info.act_acc[i + 7] = d->qacc[i + 1];
  }

  if (!isPositionControl) {
    // 扭矩控制模式下的扭矩数据更新（与原始代码完全一致）
    measured_info.act_tau[dof - 1] = d->ctrl[0];
    measured_info.act_tau[dof - 2] = d->ctrl[dof - 1];
    measured_info.act_tau[dof - 3] = d->ctrl[dof - 2];
    for (int i = 0; i < 7; ++i) {
      measured_info.act_tau[i] = d->ctrl[i + 7 + 1];
      measured_info.act_tau[i + 7] = d->ctrl[i + 1];
    }
  }

  // 仿真和实物调用相同的接口（与原始代码完全一致）
  getDriverInfo(&measured_info);
  sendDriverInfo(&command_info);

  // 更新仿真数据（与原始代码完全一致）
  if (isPositionControl) {
    // 位置控制模式
    for (int i = 0; i < 7; ++i) {
      d->qpos[i + 1] = command_info.cmd_pos[i + 7];
      d->qvel[i + 1] = command_info.cmd_vel[i + 7];
      d->qpos[i + 7 + 1] = command_info.cmd_pos[i];
      d->qvel[i + 7 + 1] = command_info.cmd_vel[i];
    }
  }

  // 更新腰部位置（与原始代码完全一致）
  d->qpos[dof - 1] = command_info.cmd_pos[dof - 2];
  d->qpos[dof - 2] = command_info.cmd_pos[dof - 3];
  d->qvel[dof - 1] = command_info.cmd_vel[dof - 2];
  d->qvel[dof - 2] = command_info.cmd_vel[dof - 3];
  d->qpos[0] = command_info.cmd_pos[dof - 1];
  d->qvel[0] = command_info.cmd_vel[dof - 1];

  if (!isPositionControl) {
    // 扭矩控制模式下的PD控制（与原始代码完全一致）
    ::Eigen::VectorXd Kp(getDof()), Kd(getDof());
    Kp = ::Eigen::VectorXd::Constant(getDof(), 0);
    Kd = ::Eigen::VectorXd::Constant(getDof(), 0);

    // PdControl(Kp, Kd, 0x0F); // 原始代码中这行被注释掉了

    // 应用扭矩控制命令（与原始代码完全一致）
    for (int i = 0; i < 7; ++i) {
      d->ctrl[i + 1] = command_info.cmd_tau[i + 7];
      d->ctrl[i + 7 + 1] = command_info.cmd_tau[i];
    }
    d->ctrl[dof - 1] = command_info.cmd_tau[dof - 2];
    d->ctrl[dof - 2] = command_info.cmd_tau[dof - 3];
    d->ctrl[0] = command_info.cmd_pos[dof - 1]; // 注意：这里使用的是cmd_pos而不是cmd_tau
  }
}

std::vector<Eigen::VectorXd> DataExchange::getForceTorqueData(const mjModel* m, mjData* d) {
  std::vector<const char*> force_sensor_names = {"TCP_L_force_sensor", "TCP_R_force_sensor"};
  std::vector<const char*> torque_sensor_names = {"TCP_R_torque_sensor", "TCP_R_torque_sensor"};
  std::vector<Eigen::VectorXd> ft_wrench(2, Eigen::VectorXd::Zero(6));

  for (int i = 0; i < force_sensor_names.size(); ++i) {
    int force_sensor_id = mj_name2id(m, mjOBJ_SENSOR, force_sensor_names[i]);
    int torque_sensor_id = mj_name2id(m, mjOBJ_SENSOR, torque_sensor_names[i]);
    mjtNum* force = d->sensordata + m->sensor_adr[force_sensor_id];
    mjtNum* torque = d->sensordata + m->sensor_adr[torque_sensor_id];
    for (int j = 0; j < 3; ++j) {
      ft_wrench[i][j] = force[j];
      ft_wrench[i][j + 3] = torque[j];
    }
  }

  return ft_wrench;
}

void DataExchange::updateSimulationTime() {
  ticks++;
  measured_info.upTime = ticks;
}

void DataExchange::updateWaistData(const mjModel* m, mjData* d) {
  measured_info.act_pos[dof - 1] = d->qpos[0];
  measured_info.act_vel[dof - 1] = d->qvel[0];
  measured_info.act_acc[dof - 1] = d->qacc[0];
}

void DataExchange::updateNeckData(const mjModel* m, mjData* d) {
  measured_info.act_pos[dof - 2] = d->qpos[dof - 1];
  measured_info.act_pos[dof - 3] = d->qpos[dof - 2];
  measured_info.act_vel[dof - 2] = d->qvel[dof - 1];
  measured_info.act_vel[dof - 3] = d->qvel[dof - 2];
  measured_info.act_acc[dof - 2] = d->qacc[dof - 1];
  measured_info.act_acc[dof - 3] = d->qacc[dof - 2];
}

void DataExchange::updateArmData(const mjModel* m, mjData* d) {
  for (int i = 0; i < 7; ++i) {
    measured_info.act_pos[i] = d->qpos[i + 7 + 1];
    measured_info.act_vel[i] = d->qvel[i + 7 + 1];
    measured_info.act_acc[i] = d->qacc[i + 7 + 1];
    measured_info.act_pos[i + 7] = d->qpos[i + 1];
    measured_info.act_vel[i + 7] = d->qvel[i + 1];
    measured_info.act_acc[i + 7] = d->qacc[i + 1];
  }
}

void DataExchange::applyPositionCommands(const mjModel* m, mjData* d) {
  for (int i = 0; i < 7; ++i) {
    d->qpos[i + 1] = command_info.cmd_pos[i + 7];
    d->qvel[i + 1] = command_info.cmd_vel[i + 7];
    d->qpos[i + 7 + 1] = command_info.cmd_pos[i];
    d->qvel[i + 7 + 1] = command_info.cmd_vel[i];
  }

  d->qpos[dof - 1] = command_info.cmd_pos[dof - 2];
  d->qpos[dof - 2] = command_info.cmd_pos[dof - 3];
  d->qvel[dof - 1] = command_info.cmd_vel[dof - 2];
  d->qvel[dof - 2] = command_info.cmd_vel[dof - 3];
  d->qpos[0] = command_info.cmd_pos[dof - 1];
  d->qvel[0] = command_info.cmd_vel[dof - 1];
}

void DataExchange::applyTorqueCommands(const mjModel* m, mjData* d) {
  ::Eigen::VectorXd Kp(getDof()), Kd(getDof());
  Kp = ::Eigen::VectorXd::Constant(getDof(), 0);
  Kd = ::Eigen::VectorXd::Constant(getDof(), 0);

  for (int i = 0; i < 7; ++i) {
    d->ctrl[i + 1] = command_info.cmd_tau[i + 7];
    d->ctrl[i + 7 + 1] = command_info.cmd_tau[i];
  }
  d->ctrl[dof - 1] = command_info.cmd_tau[dof - 2];
  d->ctrl[dof - 2] = command_info.cmd_tau[dof - 3];
  d->ctrl[0] = command_info.cmd_pos[dof - 1];
}

// 原始代码中的控制器函数（保持完全一致）
void myController(const mjModel* m, mjData* d) {
  // 创建临时DataExchange实例来保持原有功能
  static DataExchange dataExchange;
  dataExchange.exchangeData(m, d, false);
}

void myPositionController(const mjModel* m, mjData* d) {
  // 创建临时DataExchange实例来保持原有功能
  static DataExchange dataExchange;
  dataExchange.exchangeData(m, d, true);
  mju_zero(d->qvel, m->nv);  // 将速度置零
}