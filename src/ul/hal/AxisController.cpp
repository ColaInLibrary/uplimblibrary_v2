#include "AxisController.h"

namespace ul {
namespace hal {
AxisController::AxisController(const ::std::size_t& dof) : dof(dof) {}

AxisController::~AxisController() {}

::std::size_t AxisController::getDof() const { return this->dof; }
}  // namespace hal
}  // namespace ul
