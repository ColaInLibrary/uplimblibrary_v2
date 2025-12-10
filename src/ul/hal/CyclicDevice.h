#ifndef UL_HAL_CYCLICDEVICE_H
#define UL_HAL_CYCLICDEVICE_H

#include <ul/math/Real.h>
namespace ul {
namespace hal {
class CyclicDevice {
 public:
  CyclicDevice(const ::ul::math::Real& updateRate);

  virtual ~CyclicDevice();

  virtual ::ul::math::Real getUpdateRate() const;

  void setUpdateRate(const ::ul::math::Real& updateRate);

  virtual void step() = 0;

 protected:

 private:
  ::ul::math::Real updateRate;
};
}  // namespace hal
}  // namespace ul

#endif  // UL_HAL_CYCLICDEVICE_H
