/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-2-19
 * @Version       : 0.0.1
 * @File          : JointsMotion.cpp
 ******************************************************************************
 */

#include "InteractionControl.h"

namespace ul {
namespace controller {
::ul::std17::DebugShareBuffer interaction_control_buffer(5, 22);
InteractionControl::InteractionControl(::ul::hal::Coach& d, ::ul::mdl::Dynamic& r, const ::std::size_t& dof)
    : driver(d),
      robot(r),
      dof(dof),
      cmd_q(dof),
      cmd_qd(dof),
      cmd_qdd(dof),
      cmd_qddd(dof),
      act_q(dof),
      act_qd(dof),
      act_qdd(dof),
      act_qddd(dof),
      cmd_q_prev(dof),
      cmd_q_prev_protect(dof),
      ndo_p_temp(::ul::math::Vector::Zero(dof)),
      ndo_d_hat_temp(::ul::math::Vector::Zero(dof)),
      qd_filtered(::ul::math::Vector::Zero(dof)),
      cmd_tau_filtered(::ul::math::Vector::Zero(dof)),
      qd_filtered_r(::ul::math::Vector::Zero(7)),
      cmd_tau_filtered_r(::ul::math::Vector::Zero(7)),
      FT_sensor_l_filtered(::ul::math::Vector::Zero(6)),
      FT_sensor_r_filtered(::ul::math::Vector::Zero(6)),
      FT_sensor_l_initial(::ul::math::Vector::Zero(6)),
      FT_sensor_r_initial(::ul::math::Vector::Zero(6)),
      g_tool(2),
      load_mass(2),
      load_barycenter_trans(2),
      gravity_tool(2),
      tau_tool(2),
      act_qd_d(::ul::math::Vector::Zero(dof)),
      act_qdd_d(::ul::math::Vector::Zero(dof)),
      cmd_qd_d(::ul::math::Vector::Zero(dof)),
      cmd_qdd_d(::ul::math::Vector::Zero(dof)),
      act_tau_filtered(::ul::math::Vector::Zero(dof)),
      joint_torque_limit(dof),
      joint_collision_threshold(dof),
      joint_collision_flag(Eigen::Array<bool, Eigen::Dynamic, 1>::Constant(dof, false)),
      joint_tau_limit_flag(false) {
        sampling_period = driver.getUpdateRate();
        J_arm.resize(2);
        J_chain.resize(2);

        FT_sensor_l_initial << -9.34, 25.765, 8.835, 0.208, -0.117, 0.374;
        FT_sensor_r_initial << -39.410, 30.993, -30.378, -0.136, 0.403, 0.241;

        g_world << 0, 0, -9.81;

        // load_mass = {0.0, 1.0};
        // load_barycenter_trans = {::ul::math::Vector3(0, 0, 0.0), ::ul::math::Vector3(0, 0, -0.2)};

        load_mass = {0.0, 0.0};
        load_barycenter_trans = {::ul::math::Vector3(0, 0, 0.0), ::ul::math::Vector3(0, 0, 0)};

        ft_filter = std::make_unique<::ul::std17::GaussianFilter>(12, 21, 0.5);
        act_qd_filter = std::make_unique<::ul::std17::GaussianFilter>(dof, 21, 0.5);
        act_qdd_filter = std::make_unique<::ul::std17::GaussianFilter>(dof, 21, 0.5);
        cmd_qd_filter = std::make_unique<::ul::std17::GaussianFilter>(dof, 21, 0.5);
        cmd_qdd_filter = std::make_unique<::ul::std17::GaussianFilter>(dof, 21, 0.5);
        cmd_tau_filter = std::make_unique<::ul::std17::GaussianFilter>(dof, 21, 0.5);
        act_tau_filter = std::make_unique<::ul::std17::GaussianFilter>(dof, 21, 0.5);
        cmd_tau_filter_r = std::make_unique<::ul::std17::GaussianFilter>(7, 21, 0.5);

        // ft_filter = std::make_unique<::ul::std17::ButterworthFilter>(12, sampling_period, 25.0);
        // act_qd_filter = std::make_unique<::ul::std17::ButterworthFilter>(dof, sampling_period, 300.0);
        // act_qdd_filter = std::make_unique<::ul::std17::ButterworthFilter>(dof, sampling_period, 300.0);
        // cmd_qd_filter = std::make_unique<::ul::std17::ButterworthFilter>(dof, sampling_period, 300.0);
        // cmd_qdd_filter = std::make_unique<::ul::std17::ButterworthFilter>(dof, sampling_period, 300.0);
        // cmd_tau_filter = std::make_unique<::ul::std17::ButterworthFilter>(dof, sampling_period, 300.0);
        // act_tau_filter = std::make_unique<::ul::std17::ButterworthFilter>(dof, sampling_period, 300.0);
        // cmd_tau_filter_r = std::make_unique<::ul::std17::ButterworthFilter>(7, sampling_period, 500.0);

        act_q_diff = std::make_unique<::ul::std17::Differentiator>(dof, sampling_period, 2);
        cmd_q_diff = std::make_unique<::ul::std17::Differentiator>(dof, sampling_period, 2);
      }

InteractionControl::~InteractionControl() {}

void InteractionControl::setFTSensor(::ul::math::Vector& ft_l, ::ul::math::Vector& ft_r) {
  this->FT_sensor_l = ft_l;
  this->FT_sensor_r = ft_r;
}

void InteractionControl::getFTSensor(::ul::math::Vector& ft_l, ::ul::math::Vector& ft_r) {
  ft_l = this->FT_sensor_l;
  ft_r = this->FT_sensor_r;
}

::ul::math::Vector InteractionControl::getTauExt() {
  return this->tau_est;
}

::Eigen::Array<bool, Eigen::Dynamic, 1> InteractionControl::getJointCollisionFlag() {
  return this->joint_collision_flag;
}

void InteractionControl::update() {
  this->cmd_q = this->driver.getCommandJointPosition();
  this->act_q = this->driver.getJointPosition();
  this->cmd_qd = this->driver.getCommandJointVelocity();
  this->act_qd = this->driver.getJointVelocity();
  this->cmd_qdd = this->driver.getCommandJointAcceleration();
  this->act_tau = this->driver.getJointTorque();

  // 计算期望值
  this->robot.setPosition(this->cmd_q);
  this->robot.setSpeed(this->cmd_qd);
  
  this->robot.calculateMassMatrix();
  this->M_d = this->robot.getMassMatrix();

  this->robot.calculateCentrifugalCoriolis();
  this->tau_c_d = this->robot.getCentrifugalCoriolis();

  this->robot.calculateGravity();
  this->tau_g_d = this->robot.getGravity();

  this->robot.calculateFriction();
  this->tau_f_d = this->robot.getFriction();

  // 计算实际值
  this->robot.setPosition(this->act_q);
  this->robot.setSpeed(this->act_qd);

  this->robot.calculateMassMatrix();
  this->M = this->robot.getMassMatrix();

  this->robot.calculateCentrifugalCoriolis();
  this->tau_c = this->robot.getCentrifugalCoriolis();

  this->robot.calculateGravity();
  this->tau_g = this->robot.getGravity();

  this->robot.calculateFriction();
  this->tau_f = this->robot.getFriction();

  for(int i = 0; i < 2; i++) {
    if (this->robot.existFrame(this->cartesian_config.ee_name[i])) {
      this->robot.setOperationalFrameIndex(this->cartesian_config.ee_idx[i]);
      this->robot.setOperationalFrameName(this->cartesian_config.ee_name[i]);
      this->robot.calculateJacobian(0); // ::pinocchio::LOCAL
      this->robot.getJacobian();
      this->J_arm[i] = this->robot.getJacobian().block(0, this->cartesian_config.joint_idx_first[i] - 1, 6, this->cartesian_config.joint_num[i]);
      this->J_chain[i] = this->robot.getJacobian()(Eigen::all, this->urdf_chain_idx_[i]);

      this->g_tool[i] = this->robot.getOperationalTransform(this->cartesian_config.ee_idx[i]).rotation().transpose() * this->g_world;
      ::ul::math::Vector3 gravity_force_temp =  this->load_mass[i] * this->g_tool[i];
      this->gravity_tool[i].head(3) = gravity_force_temp;
      this->gravity_tool[i].tail(3) = this->load_barycenter_trans[i].cross(gravity_force_temp);
    } else {
      ::std::cout << "Frame " << this->cartesian_config.ee_name[i] << " does not exist!" << ::std::endl;
    }
  }

  // 观测器 
  this->tau_est = this->NDO();
  //////////////////////////////////////////////////////////////////////////
  // ::ul::math::Vector load_ft(12), load_t(16);

  // load_ft << this->gravity_tool[0], this->gravity_tool[1];

  // ::std::vector<::ul::math::Vector> force_tool(2);

  // force_tool[0] = this->gravity_tool[0];
  // force_tool[1] = this->gravity_tool[1];

  // this->tau_tool = fee2tau(false, force_tool);

  // load_t << this->tau_tool[0], this->tau_tool[1];

  // // interaction_control_buffer.set(1, load_ft);
  // interaction_control_buffer.set(1, load_t);

  // this->tau_est(alg_chain_idx_[0]) -= this->tau_tool[0];
  // this->tau_est(alg_chain_idx_[1]) -= this->tau_tool[1];
  //////////////////////////////////////////////////////////////////////////
  this->joint_collision_flag = (this->tau_est.cwiseAbs().array() > this->joint_collision_threshold.array());

  interaction_control_buffer.set(0, this->tau_est);
  //////////////////////////////////////////////////////////////////////////
  // ::ul::math::Vector FT_sensor_act(12), FT_sensor_filtered(12);
  // ::ul::math::Vector direction_l(6), direction_r(6);

  // direction_l << -1, 1, -1, -1, 1, -1;
  // direction_r << -1, 1, -1, -1, 1, -1;
  // FT_sensor_act << (this->FT_sensor_l - this->FT_sensor_l_initial).array() * direction_l.array(), 
  //                  (this->FT_sensor_r - this->FT_sensor_r_initial).array() * direction_r.array();

  // FT_sensor_filtered = ft_filter->update(FT_sensor_act);

  // interaction_control_buffer.set(1, FT_sensor_filtered);

  // this->FT_sensor_l_filtered << FT_sensor_filtered.segment(0,6);
  // this->FT_sensor_r_filtered << FT_sensor_filtered.segment(6,6);
  //////////////////////////////////////////////////////////////////////////
  auto result_act = act_q_diff->update(act_q);
  auto result_cmd = cmd_q_diff->update(cmd_q);

  this->act_qd_d = act_qd_filter->update(result_act[0]);
  this->act_qdd_d = act_qdd_filter->update(result_act[1]);
  // interaction_control_buffer.set(4, this->act_qd_d);

  this->cmd_qd_d = cmd_qd_filter->update(result_cmd[0]);
  this->cmd_qdd_d = cmd_qdd_filter->update(result_cmd[1]);

  act_tau_filtered = act_tau_filter->update(act_tau);
  
  ::std::vector<::ul::math::Vector> tau(2), fee(2);
  // tau[0] = this->tau_est.segment(0,this->cartesian_config.joint_num[0]);
  // tau[1] = this->tau_est.segment(this->cartesian_config.joint_num[0],this->cartesian_config.joint_num[1]);
  // fee = tau2fee(true, tau);
  //////////////////////////////////////////////////////////////////////////
  // ::ul::math::Vector f_ee(12), Tau(16);
  // // f_ee << fee[0], fee[1];
  // // interaction_control_buffer.set(0, f_ee);

  // fee[0] = this->FT_sensor_l_filtered;
  // fee[1] = this->FT_sensor_r_filtered;

  // tau = fee2tau(false, fee);
  // Tau << tau[0], tau[1];
  // interaction_control_buffer.set(2, Tau);
  //////////////////////////////////////////////////////////////////////////
}

void InteractionControl::trajectoryTrackingController() {
  ::ul::math::Vector temp_one = ::ul::math::Vector::Ones(this->dof);
  ::ul::math::Vector err_q(dof), err_qd(dof);

  err_q = cmd_q - act_q;
  err_qd = cmd_qd_d - act_qd_d;

  cmd_tau = this->M_d * cmd_qdd + this->Kp.cwiseProduct(err_q) + this->Kd.cwiseProduct(err_qd) + tau_c_d + tau_g_d + tau_f_d - tau_est; 
  // cmd_tau = this->M_d * cmd_qdd + Kp.cwiseProduct(err_q) + Kd.cwiseProduct(err_qd) + tau_c_d + tau_g_d + tau_f_d; 
  // cmd_tau = Kp.cwiseProduct(err_q) + Kd.cwiseProduct(err_qd) + tau_c_d + tau_g_d + tau_f_d; 
  // cmd_tau = Kp.cwiseProduct(err_q) + Kd.cwiseProduct(err_qd) + tau_c_d + tau_g_d + tau_f_d - tau_est; 
  // cmd_tau = Kp.cwiseProduct(err_q) + Kd.cwiseProduct(err_qd) + tau_c + tau_g - tau_est; 
  // cmd_tau = Kp.cwiseProduct(err_q) + Kd.cwiseProduct(err_qd);

  // tau_model = tau_c_d + tau_g_d + tau_f_d;
  // tau_pid = Kp.cwiseProduct(err_q) + Kd.cwiseProduct(err_qd); 

  // interaction_control_buffer.set(1, tau_model);
  // interaction_control_buffer.set(2, tau_pid);

  cmd_tau_filtered = cmd_tau_filter->update(cmd_tau);

  for (int j = 0; j < this->dof; ++j) {
    if (abs(cmd_tau_filtered[j]) > abs(joint_torque_limit[j])) {
      ::std::cout << RED << " cmd_tau[" << j << "]: " << cmd_tau_filtered[j] << " is out of limit: " << joint_torque_limit[j] << RESET << std::endl;
      joint_tau_limit_flag = true;
      break;
    }
  }

  if(joint_tau_limit_flag){  //力矩超限，进入拖动模式
    cmd_tau_filtered = tau_g;
  }

  this->driver.setJointTorque(cmd_tau_filtered);
}

void InteractionControl::impedanceController(::ul::math::Vector &kp, ::ul::math::Vector &kd) {
  ::ul::math::Vector err_q(this->dof), err_qd(this->dof), err_qdd(this->dof);

  err_q = cmd_q - act_q;
  err_qd = cmd_qd_d - act_qd_d;
  // err_qdd = cmd_qdd_d - act_qdd_d;

  // ::ul::math::Vector tau_e;

  // tau_e = this->M * err_qdd + kp.cwiseProduct(err_q) + kd.cwiseProduct(err_qd);
  // interaction_control_buffer.set(4, -tau_e);
  
  cmd_tau = this->M * cmd_qdd + kp.cwiseProduct(err_q) + kd.cwiseProduct(err_qd) + tau_c + tau_g; 
  // cmd_tau = tau_g; 

  // this->cmd_tau(alg_chain_idx_[0]) -= this->tau_tool[0];
  // this->cmd_tau(alg_chain_idx_[1]) -= this->tau_tool[1];
  
  cmd_tau_filtered = cmd_tau_filter->update(cmd_tau);

  // interaction_control_buffer.set(1, cmd_tau_filtered);

  for (int j = 0; j < this->dof; ++j) {
    if (abs(cmd_tau_filtered[j]) > abs(joint_torque_limit[j])) {
      ::std::cout << RED << " cmd_tau[" << j << "]: " << cmd_tau_filtered[j] << " is out of limit: " << joint_torque_limit[j] << RESET << std::endl;
      joint_tau_limit_flag = true;
      break;
    }
  }

  if(joint_tau_limit_flag){  //力矩超限，进入拖动模式
    cmd_tau_filtered = tau_g;
  }

  this->driver.setJointTorque(cmd_tau_filtered);
}

void InteractionControl::impedanceController() {
  this->impedanceController(this->K_impdc, this->D_impdc);
}

bool InteractionControl::protectedMode(double a_max, double delta_p_min) {
  this->cmd_tau_filter = std::make_unique<::ul::std17::GaussianFilter>(dof, 21, 0.5);
  this->cmd_q_prev_protect = this->driver.getJointPosition();

  ::ul::math::Vector v0 = this->driver.getLastCommandJointVelocity();
  
  ::ul::math::Vector delta_p = ((0.5 * v0.array().square()) / a_max).max(delta_p_min);

  this->lower_pos_limit_protect.resize(dof);
  this->upper_pos_limit_protect.resize(dof);

  this->lower_pos_limit_protect = (this->cmd_q_prev_protect - delta_p).cwiseMax(lower_pos_limit);
  this->upper_pos_limit_protect = (this->cmd_q_prev_protect + delta_p).cwiseMin(upper_pos_limit);

  ::ul::math::Real max_abs = v0.cwiseAbs().maxCoeff();
  ::ul::math::Vector qd_vector(dof);
  if (max_abs > 0.01) {
    qd_vector = v0/max_abs;
  } else {
    qd_vector.setZero();
  }
  this->delta_v = -qd_vector * a_max * this->sampling_period;
  
  return true;
}

void InteractionControl::impedanceController_protected() {
  double dt = this->sampling_period;
  /////////////////////////////////////////////////////
  ::ul::math::Vector qd_last = driver.getLastCommandJointVelocity();
  auto cond1 = qd_last.array().abs() <= this->delta_v.array().abs();
  auto cond2 = this->delta_v.array().abs() < 1e-10;  // 使用小阈值判断零
  ::ul::math::Vector qd_temp = (cond1 || cond2).select(0.0, qd_last + this->delta_v);
  ::ul::math::Vector q_temp = qd_temp * dt + driver.getLastCommandJointPosition();
  /////////////////////////////////////////////////////
  std::vector<uint8_t> cmd_mode_operation = this->driver.getCommandJointModeOperation();
  this->cmd_qd_d.setZero();
  this->cmd_qdd.setZero();

  double zone = 0.05;
  double v_max = 0.02;

  for (int j = 0; j < this->dof; ++j) {
    double q = act_q[j];
    double lower = lower_pos_limit_protect[j];
    double upper = upper_pos_limit_protect[j];
    
    double lower_start = lower + zone;   // 软区起点（向内）
    double upper_start = upper - zone;

    double cmd_prev = cmd_q_prev_protect[j];
    
    // 正常区：严格跟随
    if (q >= lower_start && q <= upper_start) {
      cmd_q[j] = q;
    }
    // 进入上软区：把 cmd_q 向内拉到 upper_start（使 cmd_q <= act_q，产生内向恢复力）
    else if (q > upper_start) {
      double s = (q - upper_start) / zone; // 0..1
      double S = 0.5 * (1.0 - cos(M_PI * std::min(1.0, std::max(0.0, s))));
      // 从 upper_start 向 q 插值（保证单调）
      double desired = (1.0 - S) * upper_start + S * q; // monotonic in q
      double max_step = v_max * dt;
      double delta = desired - cmd_prev;

      // 单向约束：act 在向外推（q > cmd_prev）时，只允许 cmd 增加（靠近上限）
      if (q > cmd_prev) {
        if (delta < 0.0) delta = 0.0;              // 禁止朝内移动
        if (delta >  max_step) delta =  max_step; // 限速朝外
        cmd_q[j] = cmd_prev + delta;
      } else {
        // act 在往内或没动：允许向内移动，但受速率限制；
        cmd_q[j] = q;
      }
    }
    // 对称：下软区
    else if (q < lower_start) {
      double s = (lower_start - q) / zone; // 0..1
      double S = 0.5 * (1.0 - cos(M_PI * std::min(1.0, std::max(0.0, s))));
      double desired = (1.0 - S) * lower_start + S * q; // monotonic in q (increasing with q)
      double max_step = v_max * dt;
      double delta = desired - cmd_prev;

      // 对下限：当 act 在向外推（q < cmd_prev）时，只允许 cmd 减小（向外）
      if (q < cmd_prev) {
        if (delta > 0.0) delta = 0.0;              // 禁止朝内移动（变大）
        if (delta < -max_step) delta = -max_step; // 限速朝外（变小）
        cmd_q[j] = cmd_prev + delta;
      } else {
        // act 在往内或没动：允许向内移动（变大），但速率受限；
        cmd_q[j] = q;
      }
    }
    cmd_q[j] = std::clamp(cmd_q[j], lower, upper);
    // 更新 prev
    cmd_q_prev_protect[j] = cmd_q[j];
  }

  // 阻抗控制
  this->impedanceController(this->K_protect, this->D_protect);

  for(size_t i = 0; i < cmd_mode_operation.size(); ++i) {
    if(cmd_mode_operation[i] != 10) {
      this->act_q[i] = q_temp[i];
      this->act_qd[i] = qd_temp[i];
    }
  }

  this->driver.setJointPosition(this->act_q);
  this->driver.setJointVelocity(this->act_qd);
  this->driver.setJointAcceleration(this->cmd_qdd);
}

::ul::math::Vector InteractionControl::NDO() {
  Eigen::LLT<Eigen::MatrixXd> llt(this->M);
  ::ul::math::Vector SysParam1;
  ::ul::math::Vector t_c_g_f = this->act_tau_filtered - this->tau_c - this->tau_g - this->tau_f;
  // ::ul::math::Vector t_c_g_f = this->act_tau_filtered - this->tau_c - this->tau_g;
  // ::ul::math::Vector t_c_g_f = this->act_tau - this->tau_c - this->tau_g - this->tau_f;
  // ::ul::math::Vector t_c_g_f = this->act_tau - this->tau_c - this->tau_g;

  if (llt.info() == Eigen::Success)
      SysParam1 = llt.solve(t_c_g_f);
  else
      SysParam1 =  this->M.completeOrthogonalDecomposition().solve(t_c_g_f); // 退化时fallback

  // 二阶微分方程
  ::ul::math::Vector p = this->ndo_p_temp + this->sampling_period * (SysParam1 + this->ndo_d_hat_temp);
  ::ul::math::Vector d_hat = ndo_gain.cwiseProduct(this->act_qd - p);
 
  // 保存最后的状态值
  this->ndo_d_hat_temp = d_hat;
  this->ndo_p_temp = p;
  
  return this->M * d_hat;
}

::std::vector<::ul::math::Vector> InteractionControl::tau2fee(bool isArm, ::std::vector<::ul::math::Vector> tau) {
  ::ul::math::Real damping = 0.05;
  bool dosvd = true;
  ::std::vector<::ul::math::Vector> fee(2);
  ::std::vector<::ul::math::Matrix> J_T(2), J_T_inv(2);
  for(int i = 0; i < 2; i++) {
    if (isArm) {
      J_T[i] = this->J_arm[i].transpose();
    } else {
      J_T[i] = this->J_chain[i].transpose();
    }
    this->robot.calculateJacobianInverse_user(J_T[i], J_T_inv[i], damping, dosvd);
    fee[i] = J_T_inv[i] * tau[i];
  }
  return fee;
}

::std::vector<::ul::math::Vector> InteractionControl::fee2tau(bool isArm, ::std::vector<::ul::math::Vector> fee) {
  ::ul::math::Real damping = 0.05;
  bool dosvd = true;
  ::std::vector<::ul::math::Vector> tau(2);
  ::std::vector<::ul::math::Matrix> J_T(2);
  for(int i = 0; i < 2; i++) {
    if (isArm) {
      J_T[i] = this->J_arm[i].transpose();
    } else {
      J_T[i] = this->J_chain[i].transpose();
    }
    tau[i] = J_T[i] * fee[i];
  }
  return tau;
}

bool InteractionControl::setControllerParas(const char *file_name, bool printout) {
  YAML::Node config = YAML::LoadFile(file_name);
  std::string label;
  label = "TORQUE_LIMIT";
  if (config[label]) {
    std::vector<double> numbers;
    numbers = config[label].as<std::vector<double>>();
    if (numbers.size() != this->dof) {
      ::std::cout << "TORQUE_LIMIT : " << numbers.size() << std::endl;
      ::std::cout << "TORQUE_LIMIT size is not equal with the dof!" << std::endl;
      return false;
    } else {
      this->joint_torque_limit.resize(this->dof);
      for (int j = 0; j < this->dof; ++j) {
        this->joint_torque_limit[j] = numbers[j];
      }
    }
    if (printout) {
      ::std::cout << "joint_torque_limit: " << this->joint_torque_limit.transpose() << std::endl;
    }
  } else {
    ::std::cout << RED << "[Error] config File:" << file_name << " has no " << label << "." << RESET << std::endl;
    return false;
  }

  label = "NDO_GAIN";
  if (config[label]) {
    std::vector<double> numbers;
    numbers = config[label].as<std::vector<double>>();
    if (numbers.size() != this->dof) {
      ::std::cout << "NDO_GAIN: " << numbers.size() << std::endl;
      ::std::cout << "NDO_GAIN size is not equal with the dof!" << std::endl;
      return false;
    } else {
      this->ndo_gain.resize(this->dof);
      for (int j = 0; j < this->dof; ++j) {
        this->ndo_gain[j] = numbers[j];
      }
    }
    if (printout) {
      ::std::cout << "ndo_gain: " << this->ndo_gain.transpose() << std::endl;
    }
  } else {
    ::std::cout << RED << "[Error] config File:" << file_name << " has no " << label << "." << RESET << std::endl;
    return false;
  }

  label = "JOINT_COLLISION_THRESHOLD";
  if (config[label]) {
    std::vector<double> numbers;
    numbers = config[label].as<std::vector<double>>();
    if (numbers.size() != this->dof) {
      ::std::cout << "joint_collision_threshold: " << numbers.size() << std::endl;
      ::std::cout << "joint_collision_threshold size is not equal with the dof!" << std::endl;
      return false;
    } else {
      this->joint_collision_threshold.resize(this->dof);
      for (int j = 0; j < this->dof; ++j)
      {
        this->joint_collision_threshold[j] = numbers[j];
      }
    }
    if (printout) {
      ::std::cout << "joint_collision_threshold: " << this->joint_collision_threshold.transpose() << std::endl;
    }
  } else {
    ::std::cout << RED << "[Error] config File:" << file_name << " has no " << label << "." << RESET << std::endl;
    return false;
  }

  label = "KP";
  if (config[label]) {
    std::vector<double> numbers;
    numbers = config[label].as<std::vector<double>>();
    if (numbers.size() != this->dof) {
      ::std::cout << "Kp: " << numbers.size() << std::endl;
      ::std::cout << "Kp size is not equal with the dof!" << std::endl;
      return false;
    } else {
      this->Kp.resize(this->dof);
      for (int j = 0; j < this->dof; ++j) {
        this->Kp[j] = numbers[j];
      }
    }
    if (printout) {
      ::std::cout << "Kp: " << this->Kp.transpose() << std::endl;
    }
  } else {
    ::std::cout << RED << "[Error] config File:" << file_name << " has no " << label << "." << RESET << std::endl;
    return false;
  }

  label = "KD";
  if (config[label]) {
    std::vector<double> numbers;
    numbers = config[label].as<std::vector<double>>();
    if (numbers.size() != this->dof) {
      ::std::cout << "KD: " << numbers.size() << std::endl;
      ::std::cout << "KD size is not equal with the dof!" << std::endl;
      return false;
    } else {
      this->Kd.resize(this->dof);
      for (int j = 0; j < this->dof; ++j)
      {
        this->Kd[j] = numbers[j];
      }
    }
    if (printout) {
      ::std::cout << "Kd: " << this->Kd.transpose() << std::endl;
    }
  } else {
    ::std::cout << RED << "[Error] config File:" << file_name << " has no " << label << "." << RESET << std::endl;
    return false;
  }

  label = "K_TEACH";
  if (config[label]) {
    std::vector<double> numbers;
    numbers = config[label].as<std::vector<double>>();
    if (numbers.size() != this->dof) {
      ::std::cout << "K_teach: " << numbers.size() << std::endl;
      ::std::cout << "K_teach size is not equal with the dof!" << std::endl;
      return false;
    } else {
      this->K_teach.resize(this->dof);
      for (int j = 0; j < this->dof; ++j) {
        this->K_teach[j] = numbers[j];
      }
    }
    if (printout) {
      ::std::cout << "K_teach: " << this->K_teach.transpose() << std::endl;
    }
  } else {
    ::std::cout << RED << "[Error] config File:" << file_name << " has no " << label << "." << RESET << std::endl;
    return false;
  }

  label = "D_TEACH";
  if (config[label]) {
    std::vector<double> numbers;
    numbers = config[label].as<std::vector<double>>();
    if (numbers.size() != this->dof) {
      ::std::cout << "D_teach: " << numbers.size() << std::endl;
      ::std::cout << "D_teach size is not equal with the dof!" << std::endl;
      return false;
    } else {
      this->D_teach.resize(this->dof);
      for (int j = 0; j < this->dof; ++j)
      {
        this->D_teach[j] = numbers[j];
      }
    }
    if (printout) {
      ::std::cout << "D_teach: " << this->D_teach.transpose() << std::endl;
    }
  } else {
    ::std::cout << RED << "[Error] config File:" << file_name << " has no " << label << "." << RESET << std::endl;
    return false;
  }

  label = "K_IMPDC";
  if (config[label]) {
    std::vector<double> numbers;
    numbers = config[label].as<std::vector<double>>();
    if (numbers.size() != this->dof) {
      ::std::cout << "K_impdc: " << numbers.size() << std::endl;
      ::std::cout << "K_impdc size is not equal with the dof!" << std::endl;
      return false;
    } else {
      this->K_impdc.resize(this->dof);
      for (int j = 0; j < this->dof; ++j) {
        this->K_impdc[j] = numbers[j];
      }
    }
    if (printout) {
      ::std::cout << "K_impdc: " << this->K_impdc.transpose() << std::endl;
    }
  } else {
    ::std::cout << RED << "[Error] config File:" << file_name << " has no " << label << "." << RESET << std::endl;
    return false;
  }

  label = "D_IMPDC";
  if (config[label]) {
    std::vector<double> numbers;
    numbers = config[label].as<std::vector<double>>();
    if (numbers.size() != this->dof) {
      ::std::cout << "D_impdc: " << numbers.size() << std::endl;
      ::std::cout << "D_impdc size is not equal with the dof!" << std::endl;
      return false;
    } else {
      this->D_impdc.resize(this->dof);
      for (int j = 0; j < this->dof; ++j)
      {
        this->D_impdc[j] = numbers[j];
      }
    }
    if (printout) {
      ::std::cout << "D_impdc: " << this->D_impdc.transpose() << std::endl;
    }
  } else {
    ::std::cout << RED << "[Error] config File:" << file_name << " has no " << label << "." << RESET << std::endl;
    return false;
  }

  label = "K_PROTECT";
  if (config[label]) {
    std::vector<double> numbers;
    numbers = config[label].as<std::vector<double>>();
    if (numbers.size() != this->dof) {
      ::std::cout << "K_protect: " << numbers.size() << std::endl;
      ::std::cout << "K_protect size is not equal with the dof!" << std::endl;
      return false;
    } else {
      this->K_protect.resize(this->dof);
      for (int j = 0; j < this->dof; ++j) {
        this->K_protect[j] = numbers[j];
      }
    }
    if (printout) {
      ::std::cout << "K_protect: " << this->K_protect.transpose() << std::endl;
    }
  } else {
    ::std::cout << RED << "[Error] config File:" << file_name << " has no " << label << "." << RESET << std::endl;
    return false;
  }

  label = "D_PROTECT";
  if (config[label]) {
    std::vector<double> numbers;
    numbers = config[label].as<std::vector<double>>();
    if (numbers.size() != this->dof) {
      ::std::cout << "D_protect: " << numbers.size() << std::endl;
      ::std::cout << "D_protect size is not equal with the dof!" << std::endl;
      return false;
    } else {
      this->D_protect.resize(this->dof);
      for (int j = 0; j < this->dof; ++j)
      {
        this->D_protect[j] = numbers[j];
      }
    }
    if (printout) {
      ::std::cout << "D_protect: " << this->D_protect.transpose() << std::endl;
    }
  } else {
    ::std::cout << RED << "[Error] config File:" << file_name << " has no " << label << "." << RESET << std::endl;
    return false;
  }

  return true;
}

bool InteractionControl::setControllerPara(::ul::controller::ParasList &para_label, ::ul::math::Vector &para, bool printout) {
  const char* ParasListStr[] = {"TORQUE_LIMIT", "NDO_GAIN", "KP", "KD", "K_TEACH", "D_TEACH"};
  if (para.size() != this->dof) {
    ::std::cout << ParasListStr[static_cast<int>(para_label)] << " size: " << para.size() << std::endl;
    ::std::cout << ParasListStr[static_cast<int>(para_label)] << " size is not equal with the dof!" << std::endl;
    return false;
  }
  switch (para_label) {
    case ParasList::TORQUE_LIMIT:
      this->joint_torque_limit = para;
      break;
    case ParasList::NDO_GAIN:
      this->ndo_gain = para;
      break;  
    case ParasList::KP:
      this->Kp = para;
      break; 
    case ParasList::KD:
      this->Kd = para;
      break;
    case ParasList::K_TEACH:
      this->K_teach = para;
      break; 
    case ParasList::D_TEACH:
      this->D_teach = para;
      break;                 
  }
  if (printout) {
    ::std::cout << ParasListStr[static_cast<int>(para_label)] << ": " << para.transpose() << std::endl;
  }
  return true;
}

bool InteractionControl::getControllerPara(::ul::controller::ParasList &para_label, ::ul::math::Vector &para) {
  switch (para_label) {
    case ParasList::TORQUE_LIMIT:
      para = this->joint_torque_limit;
      break;
    case ParasList::NDO_GAIN:
      para = this->ndo_gain;
      break;  
    case ParasList::KP:
      para = this->Kp;
      break; 
    case ParasList::KD:
      para = this->Kd;
      break;
    case ParasList::K_TEACH:
      para = this->K_teach;
      break; 
    case ParasList::D_TEACH:
      para = this->D_teach;
      break;                 
  }
  return true;
}

void InteractionControl::setCartesianConfig(::ul::std17::CartesianConfig& c) { this->cartesian_config = c; return; }

void InteractionControl::setChainIndex(::std::vector<::Eigen::VectorXi>& urdf_chain_idx_, ::std::vector<::Eigen::VectorXi>& alg_chain_idx_) { 
  this->urdf_chain_idx_ = urdf_chain_idx_; 
  this->alg_chain_idx_ = alg_chain_idx_;
  return;
}

void InteractionControl::setLoadParas(::std::vector<::ul::math::Real>& load_mass, ::std::vector<::ul::math::Vector3>& load_barycenter_trans) { 
  // this->load_mass = load_mass;
  // this->load_barycenter_trans = load_barycenter_trans;

  this->ndo_p_temp = ::ul::math::Vector::Zero(dof);
  this->ndo_d_hat_temp = ::ul::math::Vector::Zero(dof);
  this->robot.setLoadParas(load_mass, load_barycenter_trans, this->cartesian_config.ee_idx);

  return;
}

void InteractionControl::setJointPositionLimit(const ::ul::math::Vector& lower_limit, const ::ul::math::Vector& upper_limit) {
  this->lower_pos_limit = lower_limit;
  this->upper_pos_limit = upper_limit;
}

bool InteractionControl::teachMode(::ul::std17::RobotCommand& cmd, int arm_type) {
  if (::ul::std17::RobotCommand::NO_CMD == cmd) {
    ::std::cout << YELLOW << "Type " << arm_type  << " enters the teach mode!" << RESET << std::endl;
    this->cmd_tau_filter = std::make_unique<::ul::std17::GaussianFilter>(dof, 21, 0.5);
    this->cmd_q_prev = this->driver.getJointPosition();
    return true;
  } else if (::ul::std17::RobotCommand::TEACH == cmd) {
    ::std::cout << YELLOW << "Type " << arm_type  << " enters the teach mode!" << RESET << std::endl;
    return true;
  } else {
    ::std::cout << RED << "[ERROR teachMode] Never switch to teach mode during robot motion!" << RESET << ::std::endl;
    return false;
  }
}

bool InteractionControl::endTeachMode(::ul::std17::RobotCommand& cmd, int arm_type) {
  if (::ul::std17::RobotCommand::TEACH == cmd) {
    std::cout << YELLOW << "Type " << arm_type << " out the teach mode!" << RESET << std::endl;
    return true;
  } else {
    return false;
  }
}

bool InteractionControl::teachTorqueGenerate() {
  this->act_q = this->driver.getJointPosition();
  ::ul::math::Vector cmd_q_last = this->driver.getLastCommandJointPosition();
  std::vector<uint8_t> cmd_mode_operation = this->driver.getCommandJointModeOperation();
  this->cmd_qd_d.setZero();
  this->cmd_qdd.setZero();

  double zone = 0.05;
  double v_max = 0.02;
  double dt = this->sampling_period;

  for (int j = 0; j < this->dof; ++j) {
    double q = act_q[j];
    double lower = lower_pos_limit[j];
    double upper = upper_pos_limit[j];
    
    double lower_start = lower + zone;   // 软区起点（向内）
    double upper_start = upper - zone;

    double cmd_prev = cmd_q_prev[j];
    
    // 正常区：严格跟随
    if (q >= lower_start && q <= upper_start) {
      cmd_q[j] = q;
    }
    // 进入上软区：把 cmd_q 向内拉到 upper_start（使 cmd_q <= act_q，产生内向恢复力）
    else if (q > upper_start) {
      double s = (q - upper_start) / zone; // 0..1
      double S = 0.5 * (1.0 - cos(M_PI * std::min(1.0, std::max(0.0, s))));
      // 从 upper_start 向 q 插值（保证单调）
      double desired = (1.0 - S) * upper_start + S * q; // monotonic in q
      double max_step = v_max * dt;
      double delta = desired - cmd_prev;

      // 单向约束：act 在向外推（q > cmd_prev）时，只允许 cmd 增加（靠近上限）
      if (q > cmd_prev) {
        if (delta < 0.0) delta = 0.0;              // 禁止朝内移动
        if (delta >  max_step) delta =  max_step; // 限速朝外
        cmd_q[j] = cmd_prev + delta;
      } else {
        // act 在往内或没动：允许向内移动，但受速率限制；
        cmd_q[j] = q;
      }
    }
    // 对称：下软区
    else if (q < lower_start) {
      double s = (lower_start - q) / zone; // 0..1
      double S = 0.5 * (1.0 - cos(M_PI * std::min(1.0, std::max(0.0, s))));
      double desired = (1.0 - S) * lower_start + S * q; // monotonic in q (increasing with q)
      double max_step = v_max * dt;
      double delta = desired - cmd_prev;

      // 对下限：当 act 在向外推（q < cmd_prev）时，只允许 cmd 减小（向外）
      if (q < cmd_prev) {
        if (delta > 0.0) delta = 0.0;              // 禁止朝内移动（变大）
        if (delta < -max_step) delta = -max_step; // 限速朝外（变小）
        cmd_q[j] = cmd_prev + delta;
      } else {
        // act 在往内或没动：允许向内移动（变大），但速率受限；
        cmd_q[j] = q;
      }
    }
    cmd_q[j] = std::clamp(cmd_q[j], lower, upper);
    // 更新 prev
    cmd_q_prev[j] = cmd_q[j];
  }

  // interaction_control_buffer.set(1, cmd_q);
  // interaction_control_buffer.set(2, act_q);

  // 阻抗控制
  this->impedanceController(this->K_teach, this->D_teach);

  for(size_t i = 0; i < cmd_mode_operation.size(); ++i) {
    if(cmd_mode_operation[i] != 10) {
      this->act_q[i] = cmd_q_last[i];
    }
  }

  this->driver.setJointPosition(this->act_q);
  this->driver.setJointVelocity(this->cmd_qd_d);
  this->driver.setJointAcceleration(this->cmd_qdd);

  return true;
}

}  // namespace controller
}  // namespace ul