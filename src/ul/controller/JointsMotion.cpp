/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-2-19
 * @Version       : 0.0.1
 * @File          : JointsMotion.cpp
 ******************************************************************************
 */

#include "JointsMotion.h"

namespace ul {
namespace controller {
JointsMotion::JointsMotion(const ::std::size_t& dof)
    : cmd_q(dof),
      cmd_qd(dof),
      cmd_qdd(dof),
      cmd_qddd(dof),
      cmd_q_last(dof),
      cmd_qd_last(dof),
      cmd_qdd_last(dof),
      dof(dof),
      dt(0.001),
      joint_ratio(dof),
      joint_trajectory_length(dof, 0),
      max_length(0),
      moveJ_ssv(dof),
      moveJ_path(dof, 0),
      moveJ_spline(dof),
      moveJ_spline_time(dof),
      speedJ_sv(dof),
      speedStop_data(dof),
      moveJ_point_spline(dof) {
      // sys_servoJ = ::std::make_unique<PID_MSD>(dof, 80.0, 10.0, 0.0, 1.0, 10.0, 0.0, this->dt, 201, 15);
      sys_servoJ = ::std::make_unique<PID_MSD>(dof, 480.0, 10.0, 8.0, 1.0, 8.0, 0.0, this->dt, 201, 15);
      }

JointsMotion::~JointsMotion() {}

int JointsMotion::getTrajectoryLength() { return this->max_length; }

bool JointsMotion::moveJ(const ::ul::math::Vector& q, const double& speed, const double& acceleration) {
//  assert(q.size() == this->driver.getDof());

  this->max_length = 0;
  // 设置各关节的规划参数，进行各轴预规划
  for (int j = 0; j < q.size(); ++j) {
    this->moveJ_ssv[j].x0_ = this->cmd_q_last[j];
    //    this->moveJ_ssv[j].v0_ = driver.getJointVelocity()[j];
    this->moveJ_ssv[j].v0_ = 0.0;
    this->moveJ_ssv[j].xe_ = q[j];
    this->moveJ_ssv[j].ve_ = 0.0;
    this->moveJ_ssv[j].vm_ = speed;
    this->moveJ_ssv[j].am_ = acceleration;
    this->moveJ_ssv[j].jm_ = 30;
    int result = this->moveJ_ssv[j].interpolate();
    if (result == 1) {
      this->joint_trajectory_length[j] = ::std::ceil(this->moveJ_ssv[j].t() / this->dt);
    } else if (result == -1) {
      this->joint_trajectory_length[j] = 0;
    } else if (result == 0) {
      ::std::cout << RED << "[ERROR moveJ] interpolation error!" << RESET << ::std::endl;
      return false;
    }
    // if (result) {
    //   this->joint_trajectory_length[j] = ::std::ceil(this->moveJ_ssv[j].t() / this->dt);
    // } else {
    //   this->joint_trajectory_length[j] = 0;
    // }
    // ::std::cout << YELLOW << "joint " << j << ": ratio: " << this->joint_ratio[j] << ", time: " << this->moveJ_ssv[j].t() << RESET << ::std::endl;
  }

  // 计算最长轨迹长度
  for (int i = 0; i < q.size(); ++i) {
    if (this->joint_trajectory_length[i] > this->max_length) {
      this->max_length = this->joint_trajectory_length[i];
    }
  }
  if (0 == this->max_length) {
    ::std::cout << YELLOW << "No trajectory generated!" << RESET << ::std::endl;
    return false;
  }

  // 计算各关节规划参数的比例
  for (int j = 0; j < q.size(); ++j) {
    this->joint_ratio[j] = (double)this->joint_trajectory_length[j] / this->max_length;
  }

  // 多轴同步重新规划
  for (int j = 0; j < q.size(); ++j) {
    if (0 == this->joint_ratio[j]) {
      continue;
    }
    this->moveJ_ssv[j].vm_ = this->moveJ_ssv[j].vm_ * ::std::pow(this->joint_ratio[j], 1);
    this->moveJ_ssv[j].am_ = this->moveJ_ssv[j].am_ * ::std::pow(this->joint_ratio[j], 2);
    this->moveJ_ssv[j].jm_ = this->moveJ_ssv[j].jm_ * ::std::pow(this->joint_ratio[j], 3);
    if (this->moveJ_ssv[j].interpolate()) {
      this->joint_trajectory_length[j] = ::std::ceil(this->moveJ_ssv[j].t() / this->dt);
    } else {
      this->joint_trajectory_length[j] = 0;
    }
  }

//  this->cmd_q = this->driver.getLastCommandJointPosition();
//  this->cmd_qd = this->driver.getLastCommandJointVelocity();
//  this->cmd_qdd = this->driver.getLastCommandJointAcceleration();

  return true;
}

bool JointsMotion::moveJ(const std::vector<std::vector<double>>& path, const double& time) {
  // 输入的 path 格式：二维数组，第一个维度表示 path 点数，第二个维度表示每个轴的关节角
  assert(path.size() >= 2);                             // 第一个点由当前关节角确认
  assert(time > 0.0);                                   // 时间必须大于0
  ::std::vector<double> path_time(path.size() + 1, 0);  // 第一个点的时间为0，所以要多一个点

  ::ul::math::Vector cmd_q_last = this->cmd_q_last;

  // 计算各轴的三次样条参数
  this->moveJ_path.resize(path.size() + 1);
  for (int i = 0; i < dof; ++i) {
    // 更新每个轴的插值数据
    this->moveJ_path[0] = cmd_q_last[i];
    for (int j = 0; j < path.size(); ++j) {
      this->moveJ_path[j + 1] = path[j][i];
      path_time[j + 1] = time * (j + 1) / path.size();
    }
    // Boundary derivatives
    ::ul::math::Real yd0 = 0.0;
    ::ul::math::Real yd1 = 0.0;
    ::ul::math::Spline<::ul::math::Real> sp = ::ul::math::Spline<::ul::math::Real>::CubicFirst(path_time, this->moveJ_path, yd0, yd1);
    this->moveJ_spline[i] = sp;
  }
//  this->cmd_q = this->driver.getLastCommandJointPosition();
//  this->cmd_qd = this->driver.getLastCommandJointVelocity();
//  this->cmd_qdd = this->driver.getLastCommandJointAcceleration();

  return true;
}

bool JointsMotion::moveJ(const std::vector<std::vector<double>>& path, const std::vector<double>& time) {
  // 输入的 path 格式：二维数组，第一个维度表示 path 点数，第二个维度表示每个轴的关节角
  assert(path.size() >= 2);                             // 第一个点由当前关节角确认
  assert(path.size() == time.size());                   // 路径点长度与时刻长度一致
  assert(time[0] > 0.0);                                // 传入的第一个时刻应大于0.0
  assert(std::adjacent_find(time.begin(), time.end(),[](double a, double b) {return a >= b;}) == time.end()); // 检查严格递增
  ::std::vector<double> path_time(path.size() + 1, 0);  // 当前点的时间为0，所以要多一个点

  ::ul::math::Vector cmd_q_last = this->cmd_q_last;

  // 计算各轴的三次样条参数
  this->moveJ_path.resize(path.size() + 1);
  for (int i = 0; i < dof; ++i) {
    // 更新每个轴的插值数据
    this->moveJ_path[0] = cmd_q_last[i];
    for (int j = 0; j < path.size(); ++j) {
      this->moveJ_path[j + 1] = path[j][i];
      path_time[j + 1] = time[j];
    }
    // Boundary derivatives
    ::ul::math::Real yd0 = 0.0;
    ::ul::math::Real yd1 = 0.0;
    ::ul::math::Spline<::ul::math::Real> sp = ::ul::math::Spline<::ul::math::Real>::CubicFirst(path_time, this->moveJ_path, yd0, yd1);
    this->moveJ_spline[i] = sp;
  }
//  this->cmd_q = this->driver.getLastCommandJointPosition();
//  this->cmd_qd = this->driver.getLastCommandJointVelocity();
//  this->cmd_qdd = this->driver.getLastCommandJointAcceleration();

  return true;
}

bool JointsMotion::speedJ(const ::ul::math::Vector& qd, const double& acceleration, const double& time) {
  assert(time > 0.0);
  this->max_length = 0;
  ::ul::math::Vector init_qd = this->cmd_qd_last;
  ::ul::math::Vector init_qdd = this->cmd_qdd_last;

  // set speedJ_sv parameters and pre-planning
  for (int j = 0; j < this->dof; ++j) {
    this->speedJ_sv[j].reset_paras();
    this->speedJ_sv[j].v0_ = init_qd[j];
    this->speedJ_sv[j].ve_ = qd[j];
    this->speedJ_sv[j].a0_ = init_qdd[j];
    //  这个加速度的最大值也需要在内部做一下判断
    this->speedJ_sv[j].am_ = abs(acceleration);  
    if (abs(acceleration) > 50) {
     this->speedJ_sv[j].am_ = 50;
    }
    this->speedJ_sv[j].jm_ = 300;
    this->speedJ_sv[j].j0_ = 300; 
    if (this->speedJ_sv[j].interpolate()) {
      joint_trajectory_length[j] = ::std::ceil(this->speedJ_sv[j].t() / this->dt);
    } else {
      joint_trajectory_length[j] = 0;
    }
  }

  // 计算最长轨迹长度
  for (int i = 0; i < this->dof; ++i) {
    if (joint_trajectory_length[i] > this->max_length) {
      this->max_length = joint_trajectory_length[i];
    }
  }
  if (0 == this->max_length) {
    ::std::cout << YELLOW << "No trajectory generated!" << RESET << ::std::endl;
    return false;
  }

  // 计算各关节规划参数的比例
  for (int j = 0; j < this->dof; ++j) {
    this->joint_ratio[j] = (double)joint_trajectory_length[j] / this->max_length;
  }

  // 多轴同步重新规划 2025.09.13
  for (int j = 0; j < this->dof; ++j) {
    if (0 == this->joint_ratio[j]) {
      continue;
    }
    // std::cout << "\njoint[" << j << "]" << std::endl;
    this->speedJ_sv[j].replan(this->joint_ratio[j]);
    joint_trajectory_length[j] = ::std::ceil(this->speedJ_sv[j].t() / this->dt);
  }

//  this->cmd_q = this->driver.getLastCommandJointPosition();
//  this->cmd_qd = this->driver.getLastCommandJointVelocity();
//  this->cmd_qdd = this->driver.getLastCommandJointAcceleration();

  return true;
}

bool JointsMotion::speedStop(const ::ul::math::Real& a) {
  if (abs(a) < 1e-6) {
    ::std::cout << RED << "[ERROR speedStop] Param [a] is zero!" << RESET << ::std::endl;
    return false;
  }

  ::ul::math::Vector trg_qd(this->dof);
  trg_qd.setZero();
  ::std::vector<::ul::math::Real> time(this->dof, 0);
  ::ul::math::Real max_time = 0;
  for (int j = 0; j < this->dof; ++j) {
//    time[j] = abs(this->driver.getCommandJointVelocity()[j] / a);  // TODO: 等待V2修正
    if (time[j] > max_time) {
      max_time = time[j];
    }
  }
  for (int j = 0; j < this->dof; ++j) {
    this->joint_ratio[j] = (0 == max_time) ? max_time : time[j] / max_time;
//    this->speedStop_data[j].vel = this->driver.getCommandJointVelocity()[j];  // TODO: 等待V2修正
    this->speedStop_data[j].acc = a * this->joint_ratio[j];
    this->speedStop_data[j].time = max_time;
  }

  this->max_length = ceil(max_time / this->dt);

//  this->cmd_q = this->driver.getLastCommandJointPosition();
//  this->cmd_qd = this->driver.getLastCommandJointVelocity();
//  this->cmd_qdd = this->driver.getLastCommandJointAcceleration();

  return true;
}

bool JointsMotion::moveJ_Spline(const ::ul::math::Vector& q, const ::ul::math::Vector& qd, const double& freq) {
//  assert(q.size() == this->driver.getDof());
  assert(freq > 0);
  this->max_length = 0;
  
  ::ul::math::Vector init_q, init_qd, init_qdd;
  init_q = this->cmd_q_last;
  init_qd = this->cmd_qd_last;
  init_qdd = this->cmd_qdd_last;

  ::ul::math::Real yd1 = 0.0;
  ::ul::math::Real ydd1 = 0.0;
  ::ul::math::Real time = 1.0/(freq);
  time = round(time * 1000) / 1000; 
  // std::cout << BLUE << "time: " << time << RESET << std::endl;
  // 计算各轴的五次样条参数
  for (int j = 0; j < q.size(); ++j) {
    this->moveJ_point_spline[j] = ::ul::math::Polynomial<::ul::math::Real>::QuinticFirstSecond(init_q[j], q[j], init_qd[j], qd[j], init_qdd[j], ydd1, time);
  }

//  this->cmd_q = this->driver.getLastCommandJointPosition();
//  this->cmd_qd = this->driver.getLastCommandJointVelocity();
//  this->cmd_qdd = this->driver.getLastCommandJointAcceleration();

  return true;
}

bool JointsMotion::loadCSVData(::std::vector<::std::vector<::ul::math::Real>>&& data) {
  this->csv_data = std::move(data);  // 移动
  return true;
}

bool JointsMotion::CSVTrajectoryMove() {
    if (this->csv_data.size() == 0) {
      ::std::cout << RED << "[ERROR CSVTrajectoryMove] Please load the CSV data!" << RESET << std::endl;
      return false;
    }  
    if (this->csv_data[0].size() != this->dof) {
      ::std::cout << RED << "[ERROR CSVTrajectoryMove] CSV file dof is wrong!" << RESET << std::endl;
      return false;
    }
    ::ul::math::Vector q0_temp = this->cmd_q_last;
    for (int i = 0; i < this->dof; ++i) {
      if (std::abs(csv_data[0][i] - q0_temp[i]) > 1e-5) {
        ::std::cout << RED << "[ERROR CSVTrajectoryMove] Please move to the first trajectory point!" << RESET << std::endl;
        return false;
      }
    }
    this->max_length = this->csv_data.size();
    return true;
}

bool JointsMotion::servoJ(const ::ul::math::Vector& q, const ::ul::math::Real& speed, const ::ul::math::Real& acceleration, const ::ul::math::Real& time) {
//  assert(q.size() == this->driver.getDof());
  this->max_length = 0;
  
  ::ul::math::Vector init_q, init_qd, init_qdd;
  init_q = this->cmd_q_last;
  init_qd = this->cmd_qd_last;
  init_qdd = this->cmd_qdd_last;
     
  ::std::vector<::ul::math::Real> q_vec(q.data(), q.data() + q.size());
  ::std::vector<::ul::math::Real> cmd_q_last_vec(init_q.data(), init_q.data() + init_q.size());
  ::std::vector<::ul::math::Real> cmd_qd_last_vec(init_qd.data(), init_qd.data() + init_qd.size());

  sys_servoJ->load_target(q_vec, time, cmd_q_last_vec, cmd_qd_last_vec);

//  this->cmd_q = this->driver.getLastCommandJointPosition();
//  this->cmd_qd = this->driver.getLastCommandJointVelocity();
//  this->cmd_qdd = this->driver.getLastCommandJointAcceleration();

  return true;
}

bool JointsMotion::servoJ_resetParas(const ::ul::math::Real& lookahead_time, const ::ul::math::Real& gain) {
  int window_size = std::floor(lookahead_time*1000);
  double dumping = 15.0 + gain/100.0;
  sys_servoJ->resetSys();  //重置系统
  sys_servoJ->resetParas(dumping, window_size);  //一连串调用的第一次写入参数

  return true;
}

bool JointsMotion::moveJ_trajectory(const int& step) {
  for (int i = 0; i < dof; i++) {
    // if (0 == this->joint_ratio[i]) {
    if (0 == this->joint_trajectory_length[i]) {
      cmd_q[i] = this->moveJ_ssv[i].x0_;
      cmd_qd[i] = 0;
      cmd_qdd[i] = 0;
      cmd_qddd[i] = 0;
    } else {
      cmd_q[i] = this->moveJ_ssv[i].x(step * this->dt);
      cmd_qd[i] = this->moveJ_ssv[i].v(step * this->dt);
      cmd_qdd[i] = this->moveJ_ssv[i].a(step * this->dt);
      cmd_qddd[i] = this->moveJ_ssv[i].j(step * this->dt);
    }
  }
//  this->driver.setJointPosition(cmd_q);
//  this->driver.setJointVelocity(cmd_qd);
//  this->driver.setJointAcceleration(cmd_qdd);
  return true;
}

bool JointsMotion::moveJ_path_trajectory(const int& step) {
  ::ul::math::Real time = step * this->dt;
  for (int i = 0; i < dof; i++) {
    cmd_q[i] = this->moveJ_spline[i](time);
    cmd_qd[i] = this->moveJ_spline[i](time, 1);
    cmd_qdd[i] = this->moveJ_spline[i](time, 2);
  }
//  this->driver.setJointPosition(cmd_q);
//  this->driver.setJointVelocity(cmd_qd);
//  this->driver.setJointAcceleration(cmd_qdd);
  return true;
}

bool JointsMotion::speedJ_trajectory(const int& step) {
  for (int i = 0; i < this->dof; i++) {
    if (0 == this->joint_ratio[i]) {
      cmd_qd[i] = this->speedJ_sv[i].v0_;
      cmd_qdd[i] = 0;
      cmd_qddd[i] = 0;
    } else {
      cmd_qd[i] = this->speedJ_sv[i].v(step * this->dt);
      cmd_qdd[i] = this->speedJ_sv[i].a(step * this->dt);
      cmd_qddd[i] = this->speedJ_sv[i].j(step * this->dt);
    }
  }
  cmd_q = cmd_q + cmd_qd * this->dt;

  // set command position, velocity and acceleration to drivers
//  this->driver.setJointPosition(cmd_q);
//  this->driver.setJointVelocity(cmd_qd);
//  this->driver.setJointAcceleration(cmd_qdd);
  return true;
}

bool JointsMotion::speed_stop_trajectory(const int& step) {
  for (int j = 0; j < this->dof; ++j) {
    cmd_qd[j] = this->speedStop_data[j].vel - ::ul::math::sign(this->speedStop_data[j].vel) * this->speedStop_data[j].acc * step * this->dt;
    if (step * this->dt > this->speedStop_data[j].time) {
      cmd_qd[j] = 0;
    }
  }

  cmd_q = cmd_q + cmd_qd * this->dt;
  cmd_qdd = (cmd_qd - this->cmd_qd_last) / this->dt;

  // set command position, velocity and acceleration to drivers
//  this->driver.setJointPosition(cmd_q);
//  this->driver.setJointVelocity(cmd_qd);
//  this->driver.setJointAcceleration(cmd_qdd);

  return true;
}

bool JointsMotion::moveJ_spline_trajectory(const int& step) {
  ::ul::math::Real time = (step+1) * this->dt;
  for (int i = 0; i < dof; i++) {
    cmd_q[i] = this->moveJ_point_spline[i](time);
    cmd_qd[i] = this->moveJ_point_spline[i](time, 1);
    cmd_qdd[i] = this->moveJ_point_spline[i](time, 2);
  }
//  this->driver.setJointPosition(cmd_q);
//  this->driver.setJointVelocity(cmd_qd);
//  this->driver.setJointAcceleration(cmd_qdd);
  return true;
}

bool JointsMotion::csv_trajectory(const int& step) {
  if (step < 0) {
    return false;
  }
  if (step > this->csv_data.size()) {
    return false;
  }
  for (int i = 0; i < this->dof; i++) {
    cmd_q[i] = this->csv_data[step][i];
  }
  cmd_qd = (cmd_q - this->cmd_q_last) / this->dt;
  cmd_qdd = (cmd_qd - this->cmd_qd_last) / this->dt;

//  this->driver.setJointPosition(cmd_q);
//  this->driver.setJointVelocity(cmd_qd);
//  this->driver.setJointAcceleration(cmd_qdd);
  return true;
}

bool JointsMotion::servoJ_trajectoryGenerated(const int &step) {
  ::ul::math::Real time = (step+1) * this->dt;
  bool reach;

  reach = sys_servoJ->update();

  cmd_q = Eigen::Map<Eigen::VectorXd>(sys_servoJ->x.data(), sys_servoJ->x.size());
  cmd_qd = Eigen::Map<Eigen::VectorXd>(sys_servoJ->v.data(), sys_servoJ->v.size());
  cmd_qdd = Eigen::Map<Eigen::VectorXd>(sys_servoJ->a.data(), sys_servoJ->a.size());

//  this->driver.setJointPosition(cmd_q);
//  this->driver.setJointVelocity(cmd_qd);
//  this->driver.setJointAcceleration(cmd_qdd);
  
  if (reach) {
    return true;
  } else {
    return false;
  }
}

// test
  std::vector<double> JointsMotion::get_servoJ_x_t() { return sys_servoJ->x_t; };
  std::vector<double> JointsMotion::get_servoJ_x_t_f() { return sys_servoJ->x_t_f; };

}  // namespace controller
}  // namespace ul