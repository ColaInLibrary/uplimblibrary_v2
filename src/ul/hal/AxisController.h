#ifndef UL_HAL_AXISCONTROLLER_H
#define UL_HAL_AXISCONTROLLER_H

#include <iostream>
#include <ul/math/Real.h>


namespace ul {
namespace hal {
class AxisController  {
 public:
  AxisController(const ::std::size_t& dof);

  virtual ~AxisController();

  ::std::size_t getDof() const;

 protected:
 private:
  /** Degrees of freedom. */
  ::std::size_t dof;
};
}  // namespace hal
}  // namespace ul

#endif  // UL_HAL_AXISCONTROLLER_H
