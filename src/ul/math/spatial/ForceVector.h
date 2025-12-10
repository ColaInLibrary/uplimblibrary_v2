#ifndef UL_MATH_FORCEVECTOR_H
#define UL_MATH_FORCEVECTOR_H

#include <Eigen/Core>

namespace ul {
namespace math {
namespace spatial {
template <typename Scalar>
class MotionVector;

/**
 * Force vector.
 */
template <typename Scalar>
class ForceVector {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef Scalar ScalarType;

  typedef typename ::Eigen::Matrix<Scalar, 6, 1> MatrixType;

  typedef const MatrixType ConstMatrixType;

  typedef ::Eigen::Block<MatrixType, 3, 1> MomentType;

  typedef const ::Eigen::Block<ConstMatrixType, 3, 1> ConstMomentType;

  typedef ::Eigen::Block<MatrixType, 3, 1> ForceType;

  typedef const ::Eigen::Block<ConstMatrixType, 3, 1> ConstForceType;

  ForceVector() {}

  template <typename OtherDerived>
  ForceVector(const ::Eigen::MatrixBase<OtherDerived>& other) : data(other) {}

  virtual ~ForceVector() {}

  static ForceVector Zero() { return ForceVector(MatrixType::Zero()); }

  template <typename OtherScalar>
  Scalar dot(const MotionVector<OtherScalar>& other) const;

  ForceType force() { return data.template segment<3>(3); }

  ConstForceType force() const { return data.template segment<3>(3); }

  ConstMatrixType& matrix() const { return data; }

  MomentType moment() { return data.template segment<3>(0); }

  ConstMomentType moment() const { return data.template segment<3>(0); }

  template <typename OtherScalar>
  bool isApprox(const ForceVector<OtherScalar>& other,
                const typename ::Eigen::NumTraits<Scalar>::Real& prec = ::Eigen::NumTraits<Scalar>::dummy_precision()) const {
    return matrix().isApprox(other.matrix(), prec);
  }

  template <typename OtherDerived>
  ForceVector& operator=(const ::Eigen::MatrixBase<OtherDerived>& other) {
    data = other;
    return *this;
  }

  template <typename OtherScalar>
  ForceVector& operator+=(const ForceVector<OtherScalar>& other) {
    moment() += other.moment();
    force() += other.force();
    return *this;
  }

  template <typename OtherScalar>
  ForceVector& operator-=(const ForceVector<OtherScalar>& other) {
    moment() -= other.moment();
    force() -= other.force();
    return *this;
  }

  template <typename OtherScalar>
  ForceVector& operator*=(const OtherScalar& other) {
    moment() *= other;
    force() *= other;
    return *this;
  }

  template <typename OtherScalar>
  ForceVector& operator/=(const OtherScalar& other) {
    moment() /= other;
    force() /= other;
    return *this;
  }

  template <typename OtherScalar>
  ForceVector operator+(const ForceVector<OtherScalar>& other) const {
    ForceVector res;
    res.moment() = moment() + other.moment();
    res.force() = force() + other.force();
    return res;
  }

  ForceVector operator-() const {
    ForceVector res;
    res.moment() = -moment();
    res.force() = -force();
    return res;
  }

  template <typename OtherScalar>
  ForceVector operator-(const ForceVector<OtherScalar>& other) const {
    ForceVector res;
    res.moment() = moment() - other.moment();
    res.force() = force() - other.force();
    return res;
  }

  template <typename OtherScalar>
  ForceVector operator*(const OtherScalar& other) const {
    ForceVector res;
    res.moment() = moment() * other;
    res.force() = force() * other;
    return res;
  }

  template <typename OtherScalar>
  ForceVector operator/(const OtherScalar& other) const {
    ForceVector res;
    res.moment() = moment() / other;
    res.force() = force() / other;
    return res;
  }

  void setZero() {
    moment().setZero();
    force().setZero();
  }

 protected:
 private:
  MatrixType data;
};
}  // namespace spatial
}  // namespace math
}  // namespace ul

#endif  // UL_MATH_FORCEVECTOR_H
