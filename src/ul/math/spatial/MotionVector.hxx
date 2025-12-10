#ifndef UL_MATH_MOTIONVECTOR_HXX
#define UL_MATH_MOTIONVECTOR_HXX

namespace ul {
namespace math {
namespace spatial {
template <typename Scalar>
template <typename OtherScalar>
inline ForceVector<OtherScalar> MotionVector<Scalar>::cross(const ForceVector<OtherScalar>& other) const {
  ForceVector<OtherScalar> res;
  res.moment() = angular().cross(other.moment()) + linear().cross(other.force());
  res.force() = angular().cross(other.force());
  return res;
}

template <typename Scalar>
template <typename OtherScalar>
inline Scalar MotionVector<Scalar>::dot(const ForceVector<OtherScalar>& other) const {
  return angular().dot(other.moment()) + linear().dot(other.force());
}
}  // namespace spatial
}  // namespace math
}  // namespace ul

#endif  // UL_MATH_MOTIONVECTOR_HXX
