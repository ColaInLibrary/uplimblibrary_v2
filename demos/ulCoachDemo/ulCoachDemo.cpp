/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 24-01-16
 * @Version       : 0.0.1
 * @File          : ulCoachDemo.cpp
 ******************************************************************************
 */
#include <iostream>
#include <ul/math/Transform.h>
#include <ul/math/Vector.h>
#include <ul/math/SShapeVelocity.h>
#include <ul/controller/Controller.h>
#include <ul/hal/Coach.h>

int main() {
  ::ul::hal::Coach coach(6, 0.001, 0);
  ::ul::math::Real dt =coach.getUpdateRate();
  ::std::cout << "dt = " << dt << ::std::endl;
  coach.setUpdateRate(0.0005);
  dt = coach.getUpdateRate();
  ::std::cout << "dt = " << dt << ::std::endl;

  ::ul::math::Vector q(6);
  q = coach.getJointPosition();
  ::std::cout << "q = " << q.transpose() << ::std::endl;

  ::std::cout << "Mission Success!" << ::std::endl;

  return 0;
}