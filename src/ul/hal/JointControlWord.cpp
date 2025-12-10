#include "JointControlWord.h"

namespace ul {
namespace hal {
JointControlWord::JointControlWord(const ::std::size_t& dof) : AxisController(dof) {}

JointControlWord::~JointControlWord() {}
}  // namespace hal
}  // namespace ul
