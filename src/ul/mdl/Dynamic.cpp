#include "Dynamic.h"

#include <ul/math/Quaternion.h>
#include <ul/math/Rotation.h>
#include <ul/math/Spatial.h>
#include <ul/math/algorithm.h>

#include <algorithm>
#include <stack>

//#include "World.h"

namespace ul {
namespace mdl {
Dynamic::Dynamic(const std::string& urdf_path) : Kinematic(urdf_path), G(), invM(), invMx(), M(), V(), F(), ee_inertia(2) {}

Dynamic::~Dynamic() {}

void Dynamic::calculateCentrifugalCoriolis() { this->calculateCentrifugalCoriolis(this->V); }

void Dynamic::calculateCentrifugalCoriolis(::ul::math::Vector& V) {
  // V = ::pinocchio::rnea(this->model, this->data, this->q, this->qd, ::ul::math::Vector::Zero(this->getDof()));

  pinocchio::computeCoriolisMatrix(this->model, this->data, this->q, this->qd); // 计算科氏力/离心力矩阵

  V = this->data.C * this->qd;
}

void Dynamic::calculateGravity() { this->calculateGravity(this->G); }

void Dynamic::calculateGravity(::ul::math::Vector& G) {
  ::pinocchio::computeGeneralizedGravity(this->model, this->data, this->q);
  G = this->data.g;
}

void Dynamic::calculateMassMatrix() { this->calculateMassMatrix(this->M); }

void Dynamic::calculateMassMatrix(::ul::math::Matrix& M) {
  ::pinocchio::crba(this->model, this->data, this->q);
  this->data.M.triangularView<Eigen::StrictlyLower>() = this->data.M.triangularView<Eigen::StrictlyUpper>().transpose();
  ::Eigen::MatrixXd M_temp = this->data.M;
  ::Eigen::VectorXi idx_alg = this->getIndexURDFInverse();
  M = M_temp(idx_alg, idx_alg); // 重排序
}


void Dynamic::calculateMassMatrixInverse() { this->calculateMassMatrixInverse(this->invM); }

void Dynamic::calculateMassMatrixInverse(::ul::math::Matrix& invM) {
  // ::pinocchio::cholesky::decompose(this->model, this->data);
  
  // ::ul::math::Matrix invM_temp(this->dof, this->dof);

  // pinocchio::cholesky::solve(model, data, invM_temp);
  // std::cout << "************************" << std::endl;
  // std::cout << "this->data.M:\n" << this->data.M << std::endl;
  // std::cout << "invM_temp:\n" << invM_temp << std::endl;
  
  // ::Eigen::VectorXi idx_alg = this->getIndexURDFInverse();
  // invM = invM_temp(idx_alg, idx_alg); // 重排序
  
  // std::cout << "invM:\n" << invM << std::endl;
  invM = this->M.completeOrthogonalDecomposition().pseudoInverse(); // 伪逆
  // std::cout << "************************" << std::endl;
  // std::cout << "invM:\n" << invM << std::endl;
}

void Dynamic::calculateOperationalMassMatrixInverse() { this->calculateOperationalMassMatrixInverse(this->J, this->invM, this->invMx); }

void Dynamic::calculateOperationalMassMatrixInverse(const ::ul::math::Matrix& J, const ::ul::math::Matrix& invM, ::ul::math::Matrix& invMx) const {
  invMx = J * invM * J.transpose();
}

void Dynamic::calculateFriction() {
  ::ul::math::Vector tau_fric(this->dof);
  for (int j = 0; j < this->dof; ++j) {
    tau_fric[j] = f_a[j] * ::ul::math::sat_k(this->qd[j], s_k) + f_b[j] * this->qd[j] + f_c[j];
  }
  F = tau_fric;
}

void Dynamic::forwardDynamics() {
}

void Dynamic::setLoadParas(std::vector<ul::math::Real>& load_mass,
                           std::vector<ul::math::Vector3>& load_barycenter_trans, // 末端(TCP_X)下的质心位置
                           std::vector<std::size_t>& ee_id) {
  std::vector<pinocchio::Inertia> ee_load(2);
  for (int i = 0; i < 2; ++i) {
    const pinocchio::Frame &ee_frame = model.frames[ee_id[i]];
    pinocchio::JointIndex parent_joint_id = ee_frame.parentJoint;

    // 获取 TCP 相对于 parent-joint 的 SE3 (parent -> tcp)
    const pinocchio::SE3 &X_parent_to_tcp = ee_frame.placement;

    // 将 com 从 TCP 坐标系变换到 parent-joint 坐标系
    Eigen::Vector3d com_tcp = load_barycenter_trans[i]; // 质心在 ee 坐标系下（m）
    Eigen::Vector3d com_parent = X_parent_to_tcp.rotation() * com_tcp
                                 + X_parent_to_tcp.translation();

    ee_load[i] = pinocchio::Inertia(load_mass[i], com_parent, Eigen::Matrix3d::Zero());
    if (this->first) {
      this->ee_inertia[i] = this->model.inertias[parent_joint_id];
    }
    this->model.inertias[parent_joint_id] = this->ee_inertia[i] + ee_load[i];
  }
  this->first = false;
}

::ul::math::Vector Dynamic::getCentrifugalCoriolis() { return this->V(this->getIndexURDFInverse()); }

::ul::math::Vector Dynamic::getGravity() {
  return this->G(this->getIndexURDFInverse()); 
}

::ul::math::Vector Dynamic::getFriction() { return this->F(this->getIndexURDFInverse()); }

const ::ul::math::Matrix& Dynamic::getMassMatrixInverse() const { return this->invM; }

const ::ul::math::Matrix& Dynamic::getMassMatrix() const { return this->M; }

const ::ul::math::Matrix& Dynamic::getOperationalMassMatrixInverse() const { return this->invMx; }

void Dynamic::inverseDynamics() {
  ::pinocchio::rnea(this->model, this->data, this->q, this->qd, this->qdd);
  this->tau = this->data.tau;
}

void Dynamic::setJointFrictionParas(const double s_k, const ::ul::math::Vector& f_a, const ::ul::math::Vector& f_b, const ::ul::math::Vector& f_c) {
  this->s_k = s_k;
  this->f_a = f_a(this->getIndexURDF());
  this->f_b = f_b(this->getIndexURDF());
  this->f_c = f_c(this->getIndexURDF());
}

void Dynamic::update() {
  Kinematic::update();

  this->M = ::ul::math::Matrix::Identity(this->getDof(), this->getDof());
  this->V = ::ul::math::Vector::Zero(this->getDof());
  this->G = ::ul::math::Vector::Zero(this->getDof());
  this->invM = ::ul::math::Matrix::Identity(this->getDof(), this->getDof());
//  this->invMx = ::ul::math::Matrix::Identity(6 * this->getOperationalDof(), 6 * this->getOperationalDof());
}
}  // namespace mdl
}  // namespace ul
