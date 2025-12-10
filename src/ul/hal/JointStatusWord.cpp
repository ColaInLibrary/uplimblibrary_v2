#include "JointStatusWord.h"

namespace ul {
namespace hal {
JointStatusWord::JointStatusWord(const ::std::size_t& dof) : AxisController(dof) {}

JointStatusWord::~JointStatusWord() {}
}  // namespace hal
}  // namespace ul
