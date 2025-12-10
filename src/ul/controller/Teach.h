/**
 ******************************************************************************
 * @Description   :
 * @author        : Yuan Hanqing
 * @Date          : 25-5-14
 * @Version       : 0.0.1
 * @File          : Teach.h
 ******************************************************************************
 */
#ifndef UL_SRC_UL_CONTROLLER_TEACH_H_
#define UL_SRC_UL_CONTROLLER_TEACH_H_

#include <ul/hal/Coach.h>
#include <ul/std/common.h>
#include <ul/math/Vector.h>
#include <ul/mdl/Dynamic.h>
#include <ul/std/common.h>

#include <iostream>
#include <vector>


namespace ul {
namespace controller {
class Teach {
 public:

  Teach(::ul::hal::Coach& d, ::ul::mdl::Dynamic& r);

  ~Teach();

  bool teachMode(::ul::std17::RobotCommand& cmd);

  bool endTeachMode(::ul::std17::RobotCommand& cmd);

  bool teachTorqueGenerate();

 private:
  ::ul::hal::Coach& driver;

  ::ul::mdl::Dynamic& robot;
  
  ::std::size_t dof;

};
}  // namespace controller
}  // namespace ul

#endif  // UL_SRC_UL_CONTROLLER_TEACH_H_
