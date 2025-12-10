#ifndef UL_MATH_MOTIONVECTOR_H
#define UL_MATH_MOTIONVECTOR_H

#include <Eigen/Core>

namespace ul {
namespace math {
namespace spatial {
template <typename Scalar>
class ForceVector;

/**
 * Motion vector.
 */
template <typename Scalar>
class MotionVector {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef Scalar ScalarType;

  typedef typename ::Eigen::Matrix<Scalar, 6, 6> CrossType;

  typedef typename ::Eigen::Matrix<Scalar, 6, 1> MatrixType;

  typedef const MatrixType ConstMatrixType;

  typedef ::Eigen::Block<MatrixType, 3, 1> AngularType;

  typedef const ::Eigen::Block<ConstMatrixType, 3, 1> ConstAngularType;

  typedef ::Eigen::Block<MatrixType, 3, 1> LinearType;

  typedef const ::Eigen::Block<ConstMatrixType, 3, 1> ConstLinearType;

  MotionVector() {}

  template <typename OtherDerived>
  MotionVector(const ::Eigen::MatrixBase<OtherDerived>& other) : data(other) {}

  virtual ~MotionVector() {}

  static MotionVector Zero() { return MotionVector(MatrixType::Zero()); }

  AngularType angular() { return data.template segment<3>(0); }

  ConstAngularType angular() const { return data.template segment<3>(0); }

  template <typename OtherScalar>
  ForceVector<OtherScalar> cross(const ForceVector<OtherScalar>& other) const;

  MotionVector cross(const MotionVector& other) const {
    MotionVector res;
    res.angular() = angular().cross(other.angular());
    res.linear() = angular().cross(other.linear()) + linear().cross(other.angular());
    return res;
  }

  CrossType cross66Force() const {
    CrossType res;
    res.template topLeftCorner<3, 3>() = angular().cross33();
    res.template topRightCorner<3, 3>() = linear().cross33();
    res.template bottomLeftCorner<3, 3>().setZero();
    res.template bottomRightCorner<3, 3>() = angular().cross33();
    return res;
  }

  CrossType cross66Motion() const {
    CrossType res;
    res.template topLeftCorner<3, 3>() = angular().cross33();
    res.template topRightCorner<3, 3>().setZero();
    res.template bottomLeftCorner<3, 3>() = linear().cross33();
    res.template bottomRightCorner<3, 3>() = angular().cross33();
    return res;
  }

  template <typename OtherScalar>
  Scalar dot(const ForceVector<OtherScalar>& other) const;

  LinearType linear() { return data.template segment<3>(3); }

  ConstLinearType linear() const { return data.template segment<3>(3); }

  template <typename OtherScalar>
  bool isApprox(const MotionVector<OtherScalar>& other,
                const typename ::Eigen::NumTraits<Scalar>::Real& prec = ::Eigen::NumTraits<Scalar>::dummy_precision()) const {
    return matrix().isApprox(other.matrix(), prec);
  }

  ConstMatrixType& matrix() const { return data; }

  template <typename OtherDerived>
  MotionVector& operator=(const ::Eigen::MatrixBase<OtherDerived>& other) {
    data = other;
    return *this;
  }

  template <typename OtherScalar>
  MotionVector& operator+=(const MotionVector<OtherScalar>& other) {
    angular() += other.angular();
    linear() += other.linear();
    return *this;
  }

  template <typename OtherScalar>
  MotionVector& operator-=(const MotionVector<OtherScalar>& other) {
    angular() -= other.angular();
    linear() -= other.linear();
    return *this;
  }

  template <typename OtherScalar>
  MotionVector& operator*=(const OtherScalar& other) {
    angular() *= other;
    linear() *= other;
    return *this;
  }

  template <typename OtherScalar>
  MotionVector& operator/=(const OtherScalar& other) {
    angular() /= other;
    linear() /= other;
    return *this;
  }

  template <typename OtherScalar>
  MotionVector operator+(const MotionVector<OtherScalar>& other) const {
    MotionVector res;
    res.angular() = angular() + other.angular();
    res.linear() = linear() + other.linear();
    return res;
  }

  MotionVector operator-() const {
    MotionVector res;
    res.angular() = -angular();
    res.linear() = -linear();
    return res;
  }

  template <typename OtherScalar>
  MotionVector operator-(const MotionVector<OtherScalar>& other) const {
    MotionVector res;
    res.angular() = angular() - other.angular();
    res.linear() = linear() - other.linear();
    return res;
  }

  template <typename OtherScalar>
  MotionVector operator*(const OtherScalar& other) const {
    MotionVector res;
    res.angular() = angular() * other;
    res.linear() = linear() * other;
    return res;
  }

  template <typename OtherScalar>
  MotionVector operator/(const OtherScalar& other) const {
    MotionVector res;
    res.angular() = angular() / other;
    res.linear() = linear() / other;
    return res;
  }

  void setZero() {
    angular().setZero();
    linear().setZero();
  }

 protected:
 private:
  MatrixType data;
};
}  // namespace spatial
}  // namespace math
}  // namespace ul

#endif  // UL_MATH_MOTIONVECTOR_H
