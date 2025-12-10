/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-01-14
 * @Version       : 0.0.1
 * @File          : ulSShapeVelocityDemo.cpp
 ******************************************************************************
 */
#include <iostream>
#include <ul/math/Transform.h>
#include <ul/math/Vector.h>
#include <ul/math/SingleSShapeVelocity.h>
#include <ul/math/SShapeVelocity.h>
#include <ul/controller/Controller.h>
#include <ul/std/fileSave.h>

int main() {
//  ::ul::controller::Controller controller;
  ::ul::math::SShapeVelocity<::ul::math::Real> sshape{};
//  ::ul::math::Real ratio = 0.0275789;
  ::ul::math::Real ratio = 0.0234/4.7;
  sshape.x0_ = -1.2e-5;
  sshape.xe_ = 0;
  sshape.v0_ = 0;
  sshape.ve_ = 0;
  sshape.vm_ = 0.3 * pow(ratio, 1);
  sshape.am_ = 1.4 * pow(ratio, 2);
  sshape.jm_ = 30 * pow(ratio, 3);
  if (sshape.interpolate()) {
    ::std::cout << "Time: " << sshape.t() << ::std::endl;
  } else {
    ::std::cout << RED << "Mission Failed!" << RESET << ::std::endl;
    return -1;
  }

  ::std::vector<::ul::math::Vector> matrix;
  for (int i = 0; i < sshape.t()*1000; ++i) {
    ::ul::math::Vector tmp(5);
    tmp << i/1000.0, sshape.x(i/1000.0), sshape.v(i/1000.0), sshape.a(i/1000.0), sshape.j(i/1000.0);
    matrix.push_back(tmp);
  }

  ::ul::std17::vector2csv(matrix, "/home/ah/Documents/cpp_code/UplimbLibrary/data/sshape_velocity.csv");

  // Single S-shape Velocity
  ::ul::math::SingleSShapeVelocity<::ul::math::Real> single_sshape{};
  single_sshape.v0_ = 0;
  single_sshape.ve_ = 0.3;
  single_sshape.am_ = 0.5;
  single_sshape.jm_ = 30;
  if (single_sshape.interpolate()) {
    ::std::cout << "Time: " << single_sshape.t() << ::std::endl;
  } else {
    ::std::cout << RED << "Mission Failed!" << RESET << ::std::endl;
    return -1;
  }
  matrix.clear();
  for (int i = 0; i < single_sshape.t()*1000; ++i) {
    ::ul::math::Vector tmp(5);
    tmp << i/1000.0, 0, single_sshape.v(i/1000.0), single_sshape.a(i/1000.0), single_sshape.j(i/1000.0);
    matrix.push_back(tmp);
  }

  ::ul::std17::vector2csv(matrix, "/home/ah/Documents/cpp_code/UplimbLibrary/data/single_sshape_velocity.csv");

  ::std::cout << GREEN << "Mission Success!" << RESET << ::std::endl;
  return 0;
}