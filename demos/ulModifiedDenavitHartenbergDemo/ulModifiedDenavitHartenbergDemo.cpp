/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 24-01-08
 * @Version       : 0.0.1
 * @File          : ulModifiedDenavitHartenbergDemo.cpp
 ******************************************************************************
 */
#include <iostream>
#include <ul/math/Vector.h>
#include <ul/math/Transform.h>
#include <ul/math/Constants.h>

int main() {
  ::ul::math::Transform T;
  T.fromMDH(-0.075202, 0-2.069, 0, 0);
  ::std::cout << T.matrix() << ::std::endl << ::std::endl;
  T.fromMDH(0, -70*::ul::math::Constants<double>::deg2rad-0.805, 0, -M_PI_2);
  ::std::cout << T.matrix() << ::std::endl;
  return 0;
}