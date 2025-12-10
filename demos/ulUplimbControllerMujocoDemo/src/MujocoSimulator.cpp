/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-11-12
 * @Version       : 0.0.1
 * @File          : MujocoSimulator.cpp
 ******************************************************************************
 */

#include "MujocoSimulator.h"
#include "MjController.h"
#include "DataExchange.h"
#include <iostream>

// 全局DataExchange实例
DataExchange dataExchange;

MujocoSimulator::MujocoSimulator() = default;

MujocoSimulator::~MujocoSimulator() {
  shutdown();
}

bool MujocoSimulator::initialize(const char* modelPath) {
  if (!loadModel(modelPath)) {
    return false;
  }

  applyInitialKeyframe();
  printModelInfo();
  analyzeGravityTorque();

  // 初始化数据交换
  dataExchange.initialize(m_->nq);

  return true;
}

bool MujocoSimulator::loadModel(const char* modelPath) {
  char error[1000] = "";
  m_ = mj_loadXML(modelPath, nullptr, error, 1000);
  if (!m_) {
    std::cerr << "Error loading model: " << error << std::endl;
    return false;
  }

  d_ = mj_makeData(m_);
  return true;
}

void MujocoSimulator::applyInitialKeyframe() {
  if (m_->nkey > 0) {
    mju_copy(d_->qpos, m_->key_qpos, m_->nq);
    mj_forward(m_, d_);
    std::cout << "Applied keyframe 0 to current state." << std::endl;
  }
}

void MujocoSimulator::printModelInfo() {
  std::cout << "Model loaded" << std::endl;
  std::cout << "Number of DoFs: " << m_->nq << std::endl;
  std::cout << "Number of keyframes: " << m_->nkey << std::endl;
}

void MujocoSimulator::analyzeGravityTorque() {
  mj_inverse(m_, d_);
  auto gravity_torque = d_->qfrc_bias;
  std::cout << "Gravity torque: " << std::endl;
  for (int i = 0; i < m_->nv; ++i) {
    std::cout << gravity_torque[i] << " ";
  }
  std::cout << std::endl;
}

void MujocoSimulator::step() {
  // 使用扭矩控制
  dataExchange.exchangeData(m_, d_, false);
  mj_step(m_, d_);
}

void MujocoSimulator::reset() {
  mj_resetData(m_, d_);
  mj_forward(m_, d_);
}

void MujocoSimulator::shutdown() {
  if (d_) {
    mj_deleteData(d_);
    d_ = nullptr;
  }
  if (m_) {
    mj_deleteModel(m_);
    m_ = nullptr;
  }
}