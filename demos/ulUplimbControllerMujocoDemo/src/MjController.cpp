/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-11-12
 * @Version       : 0.0.1
 * @File          : Controller.cpp
 ******************************************************************************
 */

#include "MjController.h"
//#include "DataExchange.h"
#include <iostream>
#include <Eigen/Dense>

// 全局DataExchange实例
//extern DataExchange dataExchange;

// 外部函数声明
extern "C" {
extern size_t getDof();
extern void PdControl(Eigen::VectorXd& Kp, Eigen::VectorXd& Kd, int mask);
}

MjController::MjController() {
  std::cout << "Controller initialized with default torque control mode" << std::endl;
}

void MjController::torqueControl(const mjModel* m, mjData* d) {
  // 执行数据交换（扭矩控制模式）
//  dataExchange.exchangeData(m, d, false);

  // 应用PD控制（如果需要）
//  applyPDControl(m, d);
}

void MjController::positionControl(const mjModel* m, mjData* d) {
  // 执行数据交换（位置控制模式）
//  dataExchange.exchangeData(m, d, true);

  // 在位置控制模式下，将速度置零以确保稳定性
  mju_zero(d->qvel, m->nv);
}

//void Controller::applyPDControl(const mjModel* m, mjData* d) {
//  // 配置PD控制参数
//  int dof = getDof();
//  if (dof <= 0) {
//    std::cerr << "Invalid DOF value: " << dof << std::endl;
//    return;
//  }
//
//  Eigen::VectorXd Kp(dof), Kd(dof);
//
//  // 设置PD增益
//  // 这些值应该根据实际系统进行调整
//  Kp = Eigen::VectorXd::Constant(dof, 100.0);  // 比例增益
//  Kd = Eigen::VectorXd::Constant(dof, 10.0);   // 微分增益
//
//  try {
//    // 应用PD控制到所有关节（mask = 0x0F 表示所有关节）
//    PdControl(Kp, Kd, 0x0F);
//
//    // 调试信息（可选）
//    if (d->time < 1.0) {  // 只在仿真开始时打印
//      std::cout << "PD控制应用 - Kp: " << Kp[0] << ", Kd: " << Kd[0]
//                << ", 时间: " << d->time << std::endl;
//    }
//  } catch (const std::exception& e) {
//    std::cerr << "PD控制应用失败: " << e.what() << std::endl;
//  }
//}
//
//// 重力补偿控制
//void Controller::gravityCompensation(const mjModel* m, mjData* d) {
//  // 计算重力补偿扭矩
//  mj_inverse(m, d);
//
//  // 应用重力补偿到控制命令
//  // 这需要与DataExchange模块配合使用
//  applyGravityTorque(m, d);
//}
//
//void Controller::applyGravityTorque(const mjModel* m, mjData* d) {
//  // 获取重力扭矩
//  mjtNum* gravity_torque = d->qfrc_bias;
//
//  // 调试信息：显示重力扭矩
//  static int print_count = 0;
//  if (print_count < 10) {
//    std::cout << "重力扭矩: ";
//    for (int i = 0; i < m->nv; ++i) {
//      std::cout << gravity_torque[i] << " ";
//    }
//    std::cout << std::endl;
//    print_count++;
//  }
//
//  // 在实际应用中，这里应该将重力扭矩应用到控制命令中
//  // 例如：command_info.cmd_tau[i] += gravity_torque[i];
//}
//
//// 阻抗控制（扩展功能）
//void Controller::impedanceControl(const mjModel* m, mjData* d,
//                                  const Eigen::VectorXd& desired_position,
//                                  const Eigen::VectorXd& desired_velocity,
//                                  const Eigen::MatrixXd& stiffness,
//                                  const Eigen::MatrixXd& damping) {
//  if (controlMode_ != TORQUE_CONTROL) {
//    std::cerr << "阻抗控制只能在扭矩控制模式下使用" << std::endl;
//    return;
//  }
//
//  try {
//    // 计算位置误差
//    Eigen::VectorXd position_error(m->nq);
//    for (int i = 0; i < m->nq; ++i) {
//      position_error[i] = desired_position[i] - d->qpos[i];
//    }
//
//    // 计算速度误差
//    Eigen::VectorXd velocity_error(m->nv);
//    for (int i = 0; i < m->nv; ++i) {
//      velocity_error[i] = desired_velocity[i] - d->qvel[i];
//    }
//
//    // 计算阻抗控制扭矩
//    Eigen::VectorXd impedance_torque = stiffness * position_error + damping * velocity_error;
//
//    // 应用阻抗控制扭矩（需要与DataExchange模块集成）
//    applyImpedanceTorque(impedance_torque);
//
//  } catch (const std::exception& e) {
//    std::cerr << "阻抗控制计算错误: " << e.what() << std::endl;
//  }
//}
//
//void Controller::applyImpedanceTorque(const Eigen::VectorXd& impedance_torque) {
//  // 这里应该将阻抗控制扭矩应用到控制命令中
//  // 这需要与DataExchange模块的具体实现配合
//
//  std::cout << "阻抗控制扭矩应用（需要具体实现）" << std::endl;
//  // 示例：command_info.cmd_tau += impedance_torque;
//}

//// 安全检查和限制
//bool Controller::safetyCheck(const mjModel* m, mjData* d) {
//  // 检查关节位置限制
//  if (!checkJointLimits(m, d)) {
//    std::cerr << "关节位置超出安全限制！" << std::endl;
//    return false;
//  }
//
//  // 检查关节速度限制
//  if (!checkVelocityLimits(m, d)) {
//    std::cerr << "关节速度超出安全限制！" << std::endl;
//    return false;
//  }
//
//  // 检查扭矩限制
//  if (!checkTorqueLimits(m, d)) {
//    std::cerr << "关节扭矩超出安全限制！" << std::endl;
//    return false;
//  }
//
//  return true;
//}

//bool Controller::checkJointLimits(const mjModel* m, mjData* d) {
//  for (int i = 0; i < m->nq; ++i) {
//    if (m->jnt_range && m->jnt_range[2*i] != 0 && m->jnt_range[2*i+1] != 0) {
//      if (d->qpos[i] < m->jnt_range[2*i] || d->qpos[i] > m->jnt_range[2*i+1]) {
//        std::cerr << "关节 " << i << " 位置超出限制: "
//                  << d->qpos[i] << " not in ["
//                  << m->jnt_range[2*i] << ", "
//                  << m->jnt_range[2*i+1] << "]" << std::endl;
//        return false;
//      }
//    }
//  }
//  return true;
//}
//
//bool Controller::checkVelocityLimits(const mjModel* m, mjData* d) {
//  // 简单的速度限制检查
//  const mjtNum max_velocity = 5.0;  // 弧度/秒
//
//  for (int i = 0; i < m->nv; ++i) {
//    if (std::abs(d->qvel[i]) > max_velocity) {
//      std::cerr << "关节 " << i << " 速度超出限制: " << d->qvel[i] << std::endl;
//      return false;
//    }
//  }
//  return true;
//}
//
//bool Controller::checkTorqueLimits(const mjModel* m, mjData* d) {
//  // 简单的扭矩限制检查
//  const mjtNum max_torque = 100.0;  // 牛·米
//
//  for (int i = 0; i < m->nu; ++i) {
//    if (std::abs(d->ctrl[i]) > max_torque) {
//      std::cerr << "关节 " << i << " 扭矩超出限制: " << d->ctrl[i] << std::endl;
//      return false;
//    }
//  }
//  return true;
//}

//// 紧急停止功能
//void Controller::emergencyStop(mjData* d) {
//  std::cout << "紧急停止激活！" << std::endl;
//
//  // 将所有控制命令设置为零
//  mju_zero(d->ctrl, d->ncon);
//
//  // 停止所有运动
//  mju_zero(d->qvel, d->nv);
//
//  // 这里还应该通知其他模块停止运动
//  // 例如：setZeroCommands();
//}

//// 状态监控和日志
//void Controller::monitorSystemState(const mjModel* m, mjData* d) {
//  static double last_print_time = 0.0;
//
//  // 每5秒打印一次状态信息
//  if (d->time - last_print_time > 5.0) {
//    std::cout << "\n=== 系统状态监控 ===" << std::endl;
//    std::cout << "仿真时间: " << d->time << " 秒" << std::endl;
//    std::cout << "控制模式: " << (controlMode_ == TORQUE_CONTROL ? "扭矩控制" : "位置控制") << std::endl;
//    std::cout << "关节数量: " << m->nq << std::endl;
//
//    // 打印前3个关节的状态作为示例
//    for (int i = 0; i < std::min(3, m->nq); ++i) {
//      std::cout << "关节 " << i << ": 位置=" << d->qpos[i]
//                << ", 速度=" << d->qvel[i]
//                << ", 控制=" << d->ctrl[i] << std::endl;
//    }
//
//    last_print_time = d->time;
//  }
//}