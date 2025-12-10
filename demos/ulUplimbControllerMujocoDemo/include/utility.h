/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-6-25
 * @Version       : 0.0.1
 * @File          : utility.h
 ******************************************************************************
 */

#ifndef UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_UTILITY_H_
#define UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_UTILITY_H_

#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <unistd.h>
#include <pwd.h>
#include <ul/pack/UplimbController.h>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

Eigen::VectorXd qpos_to_eigen(const mjModel* m, const mjData* d) {
  Eigen::VectorXd q(d->nq);
  for (int i = 0; i < d->nq; i++) {
    q(i) = d->qpos[i];
  }
  return q;
}

void eigen_to_ctrl(mjData* d, const Eigen::VectorXd& u) {
  for (int i = 0; i < u.size(); i++) {
    d->ctrl[i] = u(i);
  }
}

// 将 MuJoCo 的速度数组转换为 Eigen 向量
Eigen::VectorXd qvel_to_eigen(const mjModel* m, const mjData* d) {
  Eigen::VectorXd qd(m->nv);
  for (int i = 0; i < m->nv; i++) {
    qd(i) = d->qvel[i];
  }
  return qd;
}

// 检查维度是否匹配
void check_dimensions(const mjModel* m, const Eigen::VectorXd& vec, const char* name) {
  if (vec.size() != m->nu) {
    printf("Error: %s dimension (%ld) != model->nu (%d)\n", name, vec.size(), m->nu);
    exit(1);
  }
}
#endif  // UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_UTILITY_H_
