#include <algorithm>
#include <stack>
#include <ul/math/Quaternion.h>
#include <ul/math/Rotation.h>
#include <ul/math/Spatial.h>

// #include "Exception.h"
// #include "JacobianInverseKinematics.h"
#include "Kinematic.h"
// #include "Prismatic.h"
//#include "Revolute.h"

namespace ul {
namespace mdl {
Kinematic::Kinematic(const std::string& urdf_path) : Model(urdf_path), invJ(), J(), Jdqd(), Jleft(), Jright() {}

Kinematic::~Kinematic() {}

void Kinematic::calculateJacobian(const bool &inWorldFrame) {
  this->J.resize(6, this->getDof());
  this->J.setZero();
  this->calculateJacobian(this->J, inWorldFrame);
}

void Kinematic::calculateJacobian(::ul::math::Matrix &J,
                                  const bool &inWorldFrame) {
  assert(J.rows() == 6);
//  assert(J.cols() == this->getDof());
//  assert(this->model.existFrame(this->ee_name));
//  auto ee_frame_idx = this->model.getFrameId(this->ee_name, ::pinocchio::FrameType::JOINT);
//  auto ee_frame_idx = this->model.getFrameId(this->ee_name);
  auto ee_frame_idx = this->getOperationalFrameIndex();
//  ::std::cout << "=================================" << this->ee_name << ::std::endl;
  if (inWorldFrame) {
    ::pinocchio::computeFrameJacobian(this->model, this->data, this->q, ee_frame_idx, ::pinocchio::LOCAL_WORLD_ALIGNED, J);
  } else {
    ::pinocchio::computeJointJacobians(this->model, this->data, this->q);
    ::pinocchio::getFrameJacobian(this->model, this->data, ee_frame_idx, ::pinocchio::LOCAL, J);
  }
}

void Kinematic::calculateJacobianDerivative(const bool &inWorldFrame) {
  this->calculateJacobianDerivative(this->Jdqd, inWorldFrame);
}

void Kinematic::calculateJacobianDerivative(::ul::math::Vector &Jdqd,
                                            const bool &inWorldFrame) {
//  ::ul::math::Vector tmp = ::ul::math::Vector::Zero(this->getDof());

//  this->setAcceleration(tmp);
  this->forwardVelocity();
  this->forwardAcceleration();
}

void Kinematic::calculateJacobianInverse(const ::ul::math::Real &lambda,
                                         const bool &doSvd) {
  this->calculateJacobianInverse(this->J, this->invJ, lambda, doSvd);
}

void Kinematic::calculateJacobianInverse(const ::ul::math::Matrix &J,
                                         ::ul::math::Matrix &invJ,
                                         const ::ul::math::Real &lambda,
                                         const bool &doSvd) const {
  if (doSvd) {
    invJ.setZero();

    ::Eigen::JacobiSVD<::ul::math::Matrix> svd(J, ::Eigen::ComputeFullU |
                                                      ::Eigen::ComputeFullV);

    ::ul::math::Real wMin = svd.singularValues().minCoeff();
    ::ul::math::Real lambdaSqr =
        wMin < static_cast<::ul::math::Real>(1.0e-9)
            ? (1 -
               ::std::pow((wMin / static_cast<::ul::math::Real>(1.0e-9)), 2)) *
                  ::std::pow(lambda, 2)
            : 0;

    for (::std::ptrdiff_t i = 0; i < svd.nonzeroSingularValues(); ++i) {
      invJ.noalias() +=
          (svd.singularValues()(i) /
           (::std::pow(svd.singularValues()(i), 2) + lambdaSqr) *
           svd.matrixV().col(i) * svd.matrixU().col(i).transpose());
    }
  } else {
    invJ = J.transpose() * // 14*6
           (J * J.transpose() + ::std::pow(lambda, 2) * ::ul::math::Matrix::Identity(6, 6)).inverse();
           // (6x14)*(14x6)
  }
}

void Kinematic::calculateJacobianInverse_user(const ::ul::math::Matrix &J,
                                         ::ul::math::Matrix &invJ,
                                         const ::ul::math::Real &lambda,
                                         const bool &doSvd) const {
  if (doSvd) {
    invJ.setZero();

    ::Eigen::JacobiSVD<::ul::math::Matrix> svd(J, ::Eigen::ComputeThinU |
                                                      ::Eigen::ComputeThinV);

    ::Eigen::MatrixXd U = svd.matrixU();
    ::Eigen::VectorXd S = svd.singularValues();
    ::Eigen::MatrixXd V = svd.matrixV();

    // 构造对角矩阵Σ+，取奇异值的倒数（非零奇异值）
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> S_plus(S.size());
    for (int i = 0; i < S.size(); ++i) {
      if (S(i) > lambda) {
        S_plus.diagonal()[i] = 1.0 / S(i);
      } else {
        S_plus.diagonal()[i] = 1.0 / lambda; // 或者根据需要处理接近零的奇异值
      //  ::std::cout << "[WARN] The robot is almost singular!" << std::endl;
      }
    }

    // 计算伪逆
    invJ = V * S_plus * U.transpose();
  }
}

::ul::math::Real Kinematic::calculateManipulabilityMeasure() const {
  return calculateManipulabilityMeasure(this->J);
}

::ul::math::Real
Kinematic::calculateManipulabilityMeasure(const ::ul::math::Matrix &J) const {
  return ::std::sqrt((J * J.transpose()).determinant());
}

void Kinematic::forwardAcceleration() {
//  for (::std::vector<Element *>::iterator i = this->elements.begin();
//       i != this->elements.end(); ++i) {
//    (*i)->forwardAcceleration();
//  }
}

void Kinematic::forwardPosition() {

}

void Kinematic::forwardVelocity() {
//  for (::std::vector<Element *>::iterator i = this->elements.begin();
//       i != this->elements.end(); ++i) {
//    (*i)->forwardVelocity();
//  }
}

const ::ul::math::Matrix &Kinematic::getJacobian() const { return this->J; }

const ::ul::math::Vector &Kinematic::getJacobianDerivative() const {
  return this->Jdqd;
}

const ::ul::math::Matrix &Kinematic::getJacobianInverse() const {
  return this->invJ;
}

bool Kinematic::isSingular(const ::ul::math::Real &lambda) const { return this->isSingular(this->J, lambda); }

bool Kinematic::isSingular(const ::ul::math::Matrix &J, const ::ul::math::Real &lambda) const {
  ::Eigen::JacobiSVD<::ul::math::Matrix> svd(J);
  return (::std::abs(svd.singularValues()(svd.singularValues().size() - 1)) > lambda)
             ? false
             : true;
}

void Kinematic::update() {
//  Metric::update();
//
//  this->invJ = ::ul::math::Matrix::Identity(this->getDof(),
//                                            6 * this->getOperationalDof());
//  this->J = ::ul::math::Matrix::Identity(6 * this->getOperationalDof(),
//                                         this->getDof());
//  this->Jdqd = ::ul::math::Vector::Zero(6 * this->getOperationalDof());
}
} // namespace mdl
} // namespace ul
