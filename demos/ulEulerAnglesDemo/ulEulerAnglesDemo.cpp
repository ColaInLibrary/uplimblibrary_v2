/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 24-12-30
 * @Version       : 0.0.1
 * @File          : ulEulerAnglesDemo.cpp
 ******************************************************************************
 */
#include <iostream>
#include <ul/math/Vector.h>
#include <ul/mdl/Revolute.h>

int main() {
  ul::mdl::Revolute joint;
  ul::math::Vector q(5);
//  q.setRandom();
  q.setOnes();
  q = q*2;
  joint.setPosition(q);
  ::std::cout << joint.getDofPosition() << ::std::endl;
  ::std::cout << q.transpose() << ::std::endl;
  ::std::cout << joint.getPosition().transpose() << ::std::endl;

  ul::math::Vector q2(4);
  q2.setOnes();
  joint.setPosition(q2);
  ::std::cout << joint.getDofPosition() << ::std::endl;
  ::std::cout << joint.max.transpose() << ::std::endl;
  ::std::cout << joint.min.transpose() << ::std::endl;
  ::std::cout << joint.distance(q, q2) << ::std::endl;

  ul::math::Vector q3(4);
  joint.interpolate(q, q2, 0.5, q3);
  ::std::cout << q3.transpose() << ::std::endl;
  return 0;
}