#ifndef UL_MATH_PLUECKERTRANSFORM_H
#define UL_MATH_PLUECKERTRANSFORM_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ul {
namespace math {
/**
 * Spatial vector algebra.
 *
 * Roy Featherstone. Rigid Body Dynamics Algorithms. Springer, New
 * York, NY, USA, 2008.
 */
namespace spatial {
template <typename Scalar>
class ArticulatedBodyInertia;

template <typename Scalar>
class ForceVector;

template <typename Scalar>
class MotionVector;

template <typename Scalar>
class RigidBodyInertia;

/**
 * Pl&uuml;cker transform.
 */
template <typename Scalar>
class PlueckerTransform {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef Scalar ScalarType;

  typedef ::Eigen::Matrix<Scalar, 6, 6> MatrixType;

  typedef ::Eigen::Transform<Scalar, 3, ::Eigen::Affine> TransformType;

  typedef const TransformType ConstTransformType;

  typedef typename TransformType::LinearPart LinearPart;

  typedef typename TransformType::ConstLinearPart ConstLinearPart;

  typedef typename TransformType::TranslationPart TranslationPart;

  typedef typename TransformType::ConstTranslationPart ConstTranslationPart;

  PlueckerTransform() {}

  template <typename OtherScalar, int Dim, int Mode, int Options>
  PlueckerTransform(const ::Eigen::Transform<OtherScalar, Dim, Mode, Options>& other) : data(other) {}

  virtual ~PlueckerTransform() {}

  static PlueckerTransform Identity() { return PlueckerTransform(TransformType::Identity()); }

  PlueckerTransform inverse() const { return PlueckerTransform(data.inverse()); }

  MatrixType inverseForce() const {
    MatrixType res;
    res.template topLeftCorner<3, 3>() = linear();
    res.template topRightCorner<3, 3>() = translation().cross33() * linear();
    res.template bottomLeftCorner<3, 3>().setZero();
    res.template bottomRightCorner<3, 3>() = linear();
    return res;
  }

  MatrixType inverseMotion() const {
    MatrixType res;
    res.template topLeftCorner<3, 3>() = linear();
    res.template topRightCorner<3, 3>().setZero();
    res.template bottomLeftCorner<3, 3>() = translation().cross33() * linear();
    res.template bottomRightCorner<3, 3>() = linear();
    return res;
  }

  template <typename OtherScalar>
  bool isApprox(const PlueckerTransform<OtherScalar>& other,
                const typename ::Eigen::NumTraits<Scalar>::Real& prec = ::Eigen::NumTraits<Scalar>::dummy_precision()) const {
    return data.isApprox(other.data, prec);
  }

  MatrixType matrixForce() const {
    MatrixType res;
    res.template topLeftCorner<3, 3>() = linear().transpose();
    res.template topRightCorner<3, 3>() = -linear().transpose() * translation().cross33();
    res.template bottomLeftCorner<3, 3>().setZero();
    res.template bottomRightCorner<3, 3>() = linear().transpose();
    return res;
  }

  MatrixType matrixMotion() const {
    MatrixType res;
    res.template topLeftCorner<3, 3>() = linear().transpose();
    res.template topRightCorner<3, 3>().setZero();
    res.template bottomLeftCorner<3, 3>() = -linear().transpose() * translation().cross33();
    res.template bottomRightCorner<3, 3>() = linear().transpose();
    return res;
  }

  template <typename OtherScalar, int Dim, int Mode, int Options>
  PlueckerTransform& operator=(const ::Eigen::Transform<OtherScalar, Dim, Mode, Options>& other) {
    data = other;
    return *this;
  }

  template <typename OtherScalar, int Dim, int Mode, int Options>
  ::Eigen::Transform<OtherScalar, Dim, Mode, Options> operator*(const ::Eigen::Transform<OtherScalar, Dim, Mode, Options>& other) const {
    return data * other;
  }

  template <typename OtherScalar>
  ForceVector<OtherScalar> operator*(const ForceVector<OtherScalar>& other) const;

  template <typename OtherScalar>
  MotionVector<OtherScalar> operator*(const MotionVector<OtherScalar>& other) const;

  template <typename OtherScalar>
  PlueckerTransform operator*(const PlueckerTransform<OtherScalar>& other) const {
    return PlueckerTransform(data * other.data);
  }

  template <typename OtherScalar>
  RigidBodyInertia<OtherScalar> operator*(const RigidBodyInertia<OtherScalar>& other) const;

  template <typename OtherScalar>
  ArticulatedBodyInertia<OtherScalar> operator*(const ArticulatedBodyInertia<OtherScalar>& other) const;

  template <typename OtherScalar>
  ForceVector<OtherScalar> operator/(const ForceVector<OtherScalar>& other) const;

  template <typename OtherScalar>
  MotionVector<OtherScalar> operator/(const MotionVector<OtherScalar>& other) const;

  template <typename OtherScalar>
  RigidBodyInertia<OtherScalar> operator/(const RigidBodyInertia<OtherScalar>& other) const;

  template <typename OtherScalar>
  ArticulatedBodyInertia<OtherScalar> operator/(const ArticulatedBodyInertia<OtherScalar>& other) const;

  LinearPart linear() { return data.linear(); }

  ConstLinearPart linear() const { return data.linear(); }

  void setIdentity() { data.setIdentity(); }

  TransformType& transform() { return data; }

  ConstTransformType& transform() const { return data; }

  TranslationPart translation() { return data.translation(); }

  ConstTranslationPart translation() const { return data.translation(); }

 protected:
 private:
  TransformType data;
};
}  // namespace spatial
}  // namespace math
}  // namespace ul

#endif  // UL_MATH_PLUECKERTRANSFORM_H
