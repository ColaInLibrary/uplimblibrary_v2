#ifndef UL_MATH_RIGIDBODYINERTIA_HXX
#define UL_MATH_RIGIDBODYINERTIA_HXX

namespace ul {
namespace math {
namespace spatial {
template <typename Scalar>
template <typename OtherScalar>
inline ForceVector<Scalar> RigidBodyInertia<Scalar>::operator*(const MotionVector<OtherScalar>& other) const {
  ForceVector<Scalar> res;
  res.moment() = inertia() * other.angular() + cog().cross(other.linear());
  res.force() = mass() * other.linear() - cog().cross(other.angular());
  return res;
}
}  // namespace spatial
}  // namespace math
}  // namespace ul

#endif  // UL_MATH_RIGIDBODYINERTIA_HXX
