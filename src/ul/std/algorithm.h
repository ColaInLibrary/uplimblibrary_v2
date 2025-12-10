#ifndef UL_STD_ALGORITHM_H
#define UL_STD_ALGORITHM_H

#include <algorithm>

#ifndef __cpp_lib_clamp
#include <cassert>
#include <functional>
#endif

namespace ul {
namespace std17 {
#ifdef __cpp_lib_clamp
using ::std::clamp;
#else
template <typename T, typename Compare>
inline const T& clamp(const T& v, const T& lo, const T& hi, Compare comp) {
  assert(!comp(hi, lo));
  return comp(v, lo) ? lo : comp(hi, v) ? hi : v;
}

template <typename T>
inline const T& clamp(const T& v, const T& lo, const T& hi) {
  return clamp(v, lo, hi, ::std::less<T>());
}
#endif
}  // namespace std17
}  // namespace ul

#endif  // UL_STD_ALGORITHM_H
