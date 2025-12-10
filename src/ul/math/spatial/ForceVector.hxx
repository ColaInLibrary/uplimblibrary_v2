#ifndef UL_MATH_FORCEVECTOR_HXX
#define UL_MATH_FORCEVECTOR_HXX

namespace ul {
namespace math {
namespace spatial {
template <typename Scalar>
template <typename OtherScalar>
inline Scalar ForceVector<Scalar>::dot(const MotionVector<OtherScalar>& other) const {
  return moment().dot(other.angular()) + force().dot(other.linear());
}
}  // namespace spatial
}  // namespace math
}  // namespace ul

#endif  // UL_MATH_FORCEVECTOR_HXX
