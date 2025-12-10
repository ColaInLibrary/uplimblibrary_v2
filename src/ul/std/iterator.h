#ifndef UL_STD_ITERATOR_H
#define UL_STD_ITERATOR_H

#include <iterator>

namespace ul {
namespace std17 {
#ifdef __cpp_lib_nonmember_container_access
using ::std::size;
#else
template <typename T>
constexpr auto size(const T& t) -> decltype(t.size()) {
  return t.size();
}

template <typename T, ::std::size_t N>
constexpr ::std::size_t size(const T (&t)[N]) {
  return N;
}
#endif
}  // namespace std17
}  // namespace ul

#endif  // UL_STD_ITERATOR_H
