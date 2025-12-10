#ifndef UL_MATH_ALGORITHM_H
#define UL_MATH_ALGORITHM_H

namespace ul {
namespace math {
template <typename T>
inline T sign(const T& arg) {
  if (arg > 0) {
    return 1;
  } else if (arg < 0) {
    return -1;
  } else {
    return 0;
  }
}

template <typename T>
inline T sat_k(const T& arg_q, const T& arg_k) {
  T d = abs(arg_q/arg_k);
  if (d <= 1) return arg_q/arg_k;
  else return sign(arg_q/arg_k);
}
}  // namespace math
}  // namespace ul

#endif  // UL_MATH_ALGORITHM_H