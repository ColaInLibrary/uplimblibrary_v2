#ifndef UL_MATH_RIGIDBODYINERTIA_H
#define UL_MATH_RIGIDBODYINERTIA_H

#include <Eigen/Core>

namespace ul {
namespace math {
namespace spatial {
template <typename Scalar>
class ForceVector;

template <typename Scalar>
class MotionVector;

/**
 * Rigid-body inertia.
 */
template <typename Scalar>
class RigidBodyInertia {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef Scalar ScalarType;

  typedef ::Eigen::Matrix<Scalar, 6, 6> MatrixType;

  typedef ::Eigen::Matrix<Scalar, 3, 1> CenterOfGravityType;

  typedef const CenterOfGravityType ConstCenterOfGravityType;

  typedef ::Eigen::Matrix<Scalar, 3, 3> InertiaType;

  typedef const InertiaType ConstInertiaType;

  typedef Scalar MassType;

  typedef const MassType ConstMassType;

  RigidBodyInertia() {}

  RigidBodyInertia(const CenterOfGravityType& centerOfGravity, const InertiaType& inertia, const MassType& mass)
      : centerOfGravityData(centerOfGravity), inertiaData(inertia), massData(mass) {}

  template <typename OtherDerived>
  RigidBodyInertia(const ::Eigen::DenseBase<OtherDerived>& other)
      : centerOfGravityData(other.template topRightCorner<3, 3>().cross3()),
        inertiaData(other.template topLeftCorner<3, 3>()),
        massData(other.template bottomRightCorner<3, 3>().diagonal().mean()) {}

  virtual ~RigidBodyInertia() {}

  static RigidBodyInertia Identity() { return RigidBodyInertia(CenterOfGravityType::Zero(), InertiaType::Identity(), 1); }

  static RigidBodyInertia Zero() { return RigidBodyInertia(CenterOfGravityType::Zero(), InertiaType::Zero(), 0); }

  CenterOfGravityType& cog() { return centerOfGravityData; }

  ConstCenterOfGravityType& cog() const { return centerOfGravityData; }

  InertiaType& inertia() { return inertiaData; }

  ConstInertiaType& inertia() const { return inertiaData; }

  template <typename OtherScalar>
  bool isApprox(const RigidBodyInertia<OtherScalar>& other,
                const typename ::Eigen::NumTraits<Scalar>::Real& prec = ::Eigen::NumTraits<Scalar>::dummy_precision()) const {
    return cog().isApprox(other.cog(), prec) && inertia().isApprox(other.inertia(), prec) && ::Eigen::internal::isApprox(mass(), other.mass(), prec);
  }

  MassType& mass() { return massData; }

  ConstMassType& mass() const { return massData; }

  MatrixType matrix() const {
    MatrixType res;
    res.template topLeftCorner<3, 3>() = inertia();
    res.template topRightCorner<3, 3>() = cog().cross33();
    res.template bottomLeftCorner<3, 3>() = -cog().cross33();
    res.template bottomRightCorner<3, 3>() = ::Eigen::Matrix<Scalar, 3, 3>::Identity() * mass();
    return res;
  }

  template <typename OtherDerived>
  RigidBodyInertia& operator=(const ::Eigen::MatrixBase<OtherDerived>& other) {
    cog() = other.template topRightCorner<3, 3>().cross3();
    inertia() = other.template topLeftCorner<3, 3>();
    mass() = other.template bottomRightCorner<3, 3>().diagonal().mean();
    return *this;
  }

  template <typename OtherScalar>
  RigidBodyInertia& operator+=(const RigidBodyInertia<OtherScalar>& other) {
    cog() += other.cog();
    inertia() += other.inertia();
    mass() += other.mass();
    return *this;
  }

  template <typename OtherScalar>
  RigidBodyInertia& operator-=(const RigidBodyInertia<OtherScalar>& other) {
    cog() -= other.cog();
    inertia() -= other.inertia();
    mass() -= other.mass();
    return *this;
  }

  template <typename OtherScalar>
  RigidBodyInertia& operator*=(const OtherScalar& other) {
    cog() *= other;
    inertia() *= other;
    mass() *= other;
    return *this;
  }

  template <typename OtherScalar>
  RigidBodyInertia& operator/=(const OtherScalar& other) {
    cog() /= other;
    inertia() /= other;
    mass() /= other;
    return *this;
  }

  template <typename OtherScalar>
  RigidBodyInertia operator+(const RigidBodyInertia<OtherScalar>& other) const {
    RigidBodyInertia res;
    res.cog() = cog() + other.cog();
    res.inertia() = inertia() + other.inertia();
    res.mass() = mass() + other.mass();
    return res;
  }

  template <typename OtherScalar>
  RigidBodyInertia operator-(const RigidBodyInertia<OtherScalar>& other) const {
    RigidBodyInertia res;
    res.cog() = cog() - other.cog();
    res.inertia() = inertia() - other.inertia();
    res.mass() = mass() - other.mass();
    return res;
  }

  template <typename OtherScalar>
  RigidBodyInertia operator*(const OtherScalar& other) const {
    RigidBodyInertia res;
    res.cog() = cog() * other;
    res.inertia() = inertia() * other;
    res.mass() = mass() * other;
    return res;
  }

  template <typename OtherScalar>
  ForceVector<Scalar> operator*(const MotionVector<OtherScalar>& other) const;

  template <typename OtherScalar>
  RigidBodyInertia operator/(const OtherScalar& other) const {
    RigidBodyInertia res;
    res.cog() = cog() / other;
    res.inertia() = inertia() / other;
    res.mass() = mass() / other;
    return res;
  }

  void setIdentity() {
    cog().setZero();
    inertia().setIdentity();
    mass() = 1;
  }

  void setZero() {
    cog().setZero();
    inertia().setZero();
    mass() = 0;
  }

 protected:
 private:
  CenterOfGravityType centerOfGravityData;

  InertiaType inertiaData;

  MassType massData;
};
}  // namespace spatial
}  // namespace math
}  // namespace ul

#endif  // UL_MATH_RIGIDBODYINERTIA_H
