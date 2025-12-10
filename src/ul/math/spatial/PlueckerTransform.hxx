#ifndef UL_MATH_PLUECKERTRANSFORM_HXX
#define UL_MATH_PLUECKERTRANSFORM_HXX

namespace ul {
namespace math {
namespace spatial {
template <typename Scalar>
template <typename OtherScalar>
inline ForceVector<OtherScalar> PlueckerTransform<Scalar>::operator*(const ForceVector<OtherScalar>& other) const {
  ForceVector<OtherScalar> res;
  res.moment() = linear().transpose() * (other.moment() - translation().cross(other.force()));
  res.force() = linear().transpose() * other.force();
  return res;
}

template <typename Scalar>
template <typename OtherScalar>
inline MotionVector<OtherScalar> PlueckerTransform<Scalar>::operator*(const MotionVector<OtherScalar>& other) const {
  MotionVector<OtherScalar> res;
  res.angular() = linear().transpose() * other.angular();
  res.linear() = linear().transpose() * (other.linear() - translation().cross(other.angular()));
  return res;
}

template <typename Scalar>
template <typename OtherScalar>
inline RigidBodyInertia<OtherScalar> PlueckerTransform<Scalar>::operator*(const RigidBodyInertia<OtherScalar>& other) const {
  RigidBodyInertia<OtherScalar> res;
  ::Eigen::Matrix<Scalar, 3, 3> px = translation().cross33();
  ::Eigen::Matrix<Scalar, 3, 1> c_m_p = other.cog() - other.mass() * translation();
  res.cog() = linear().transpose() * c_m_p;
  res.inertia() = linear().transpose() * (other.inertia() + px * other.cog().cross33() + c_m_p.cross33() * px) * linear();
  res.mass() = other.mass();
  return res;
}

template <typename Scalar>
template <typename OtherScalar>
inline ArticulatedBodyInertia<OtherScalar> PlueckerTransform<Scalar>::operator*(const ArticulatedBodyInertia<OtherScalar>& other) const {
  ArticulatedBodyInertia<OtherScalar> res;
  ::Eigen::Matrix<Scalar, 3, 3> px = translation().cross33();
  ::Eigen::Matrix<Scalar, 3, 3> c_px_m = other.cog() - px * other.mass();
  res.cog() = linear().transpose() * c_px_m * linear();
  res.inertia() = linear().transpose() * (other.inertia() - px * other.cog().transpose() + c_px_m * px) * linear();
  res.mass() = linear().transpose() * other.mass() * linear();
  return res;
}

template <typename Scalar>
template <typename OtherScalar>
inline ForceVector<OtherScalar> PlueckerTransform<Scalar>::operator/(const ForceVector<OtherScalar>& other) const {
  ForceVector<OtherScalar> res;
  res.moment() = linear() * other.moment() + translation().cross(linear() * other.force());
  res.force() = linear() * other.force();
  return res;
}

template <typename Scalar>
template <typename OtherScalar>
inline MotionVector<OtherScalar> PlueckerTransform<Scalar>::operator/(const MotionVector<OtherScalar>& other) const {
  MotionVector<OtherScalar> res;
  res.angular() = linear() * other.angular();
  res.linear() = linear() * other.linear() + translation().cross(linear() * other.angular());
  return res;
}

template <typename Scalar>
template <typename OtherScalar>
inline RigidBodyInertia<OtherScalar> PlueckerTransform<Scalar>::operator/(const RigidBodyInertia<OtherScalar>& other) const {
  RigidBodyInertia<OtherScalar> res;
  ::Eigen::Matrix<Scalar, 3, 3> px = translation().cross33();
  ::Eigen::Matrix<Scalar, 3, 1> R_c = linear() * other.cog();
  ::Eigen::Matrix<Scalar, 3, 1> R_c_m_p = R_c + other.mass() * translation();
  res.cog() = R_c_m_p;
  res.inertia() = linear() * other.inertia() * linear().transpose() - px * R_c.cross33() - R_c_m_p.cross33() * px;
  res.mass() = other.mass();
  return res;
}

template <typename Scalar>
template <typename OtherScalar>
inline ArticulatedBodyInertia<OtherScalar> PlueckerTransform<Scalar>::operator/(const ArticulatedBodyInertia<OtherScalar>& other) const {
  ArticulatedBodyInertia<OtherScalar> res;
  typename ArticulatedBodyInertia<OtherScalar>::CenterOfGravityType c = linear() * other.cog() * linear().transpose();
  typename ArticulatedBodyInertia<OtherScalar>::MassType m = linear() * other.mass() * linear().transpose();
  ::Eigen::Matrix<Scalar, 3, 3> px = translation().cross33();
  ::Eigen::Matrix<Scalar, 3, 3> c_px_m = c + px * m;
  res.cog() = c_px_m;
  res.inertia() = linear() * other.inertia() * linear().transpose() + px * c.transpose() - c_px_m * px;
  res.mass() = m;
  return res;
}
}  // namespace spatial
}  // namespace math
}  // namespace ul

#endif  // UL_MATH_PLUECKERTRANSFORM_HXX
