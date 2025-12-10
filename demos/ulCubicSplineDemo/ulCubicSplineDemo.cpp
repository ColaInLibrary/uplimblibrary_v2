/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 24-01-09
 * @Version       : 0.0.1
 * @File          : ulCubicSplineDemo.cpp
 ******************************************************************************
 */
#include <iostream>
#include <ul/math/Vector.h>
#include <ul/math/Spline.h>

int main() {
  // 单一自由度的三次样条插值示例
  // Data points (x, y)
  std::vector<double> x = {0.0, 1.0, 2.0, 3.0};
  std::vector<double> y = {1.0, 0.0, 0.0, 2.0};

  // Boundary derivatives
  double yd0 = 0.0; // derivative at x=0
  double yd1 = 0.0; // derivative at x=3

  // Create the cubic spline
  ::ul::math::Spline<::ul::math::Real> spline = ::ul::math::Spline<::ul::math::Real>::CubicFirst(x, y, yd0, yd1);
  ::std::cout << "spline.size = " << spline.size() << ::std::endl << ::std::endl;
  for (int i = 0; i < spline.size(); ++i) {
    ::std::cout << spline.at(i).coefficient(0) << ::std::endl;
    ::std::cout << spline.at(i).coefficient(1) << ::std::endl;
    ::std::cout << spline.at(i).coefficient(2) << ::std::endl;
    ::std::cout << spline.at(i).coefficient(3) << ::std::endl;
    ::std::cout << ::std::endl;
  }
  ::ul::math::Spline<::ul::math::Real> dspline = spline.derivative();
  for (int i = 0; i < dspline.size(); ++i) {
    ::std::cout << dspline.at(i).coefficient(0) << ::std::endl;
    ::std::cout << dspline.at(i).coefficient(1) << ::std::endl;
    ::std::cout << dspline.at(i).coefficient(2) << ::std::endl;
    ::std::cout << dspline.at(i).coefficient(3) << ::std::endl;
    ::std::cout << ::std::endl;
  }

  ::ul::math::Spline<::ul::math::Real> ddspline = dspline.derivative();
  for (int i = 0; i < ddspline.size(); ++i) {
    ::std::cout << ddspline.at(i).coefficient(0) << ::std::endl;
    ::std::cout << ddspline.at(i).coefficient(1) << ::std::endl;
    ::std::cout << ddspline.at(i).coefficient(2) << ::std::endl;
    ::std::cout << ddspline.at(i).coefficient(3) << ::std::endl;
    ::std::cout << ::std::endl;
  }

  ::std::cout << "test size: " << spline.size() << ::std::endl;
  ::std::cout << "test: " << spline(2) << ::std::endl;
  ::std::cout << "test: " << spline(2, 1) << ::std::endl;
  ::std::cout << "test: " << dspline(2) << ::std::endl;
  ::std::cout << "test: " << spline(3, 2) << ::std::endl;
  ::std::cout << "test: " << ddspline(3) << ::std::endl;

//  auto minmax = spline.getMinimumMaximum();
//  double minValue = minmax.first;
//  double maxValue = minmax.second;

//  std::cout << "Spline Minimum: " << minValue << std::endl;
//  std::cout << "Spline Maximum: " << maxValue << std::endl;
  // 多自由度的三次样条插值示例
  int numJoints = 3;
  ::std::vector<::ul::math::Spline<::ul::math::Real>> splines(numJoints);
  for (int i = 0; i < numJoints; ++i) {
    // Data points (x, y)
    x = {0.0, 1.0, 2.0, 3.0};
    y = {1.0 * i, 2.0 * i, 0.0 * i, 2.0 * i};

    // Boundary derivatives
    yd0 = 0.0; // derivative at x=0
    yd1 = 0.0; // derivative at x=3
    ::ul::math::Spline<::ul::math::Real> sp = ::ul::math::Spline<::ul::math::Real>::CubicFirst(x, y, yd0, yd1);
    splines[i] = sp;
  }

  for (auto sp : splines) {
    ::std::cout << "-------------" << ::std::endl;
    for (int i = 0; i < sp.size(); ++i) {
      ::std::cout << sp.at(i).coefficient(0) << ::std::endl;
      ::std::cout << sp.at(i).coefficient(1) << ::std::endl;
      ::std::cout << sp.at(i).coefficient(2) << ::std::endl;
      ::std::cout << sp.at(i).coefficient(3) << ::std::endl;
      ::std::cout << ::std::endl;
    }
//    auto minmax = sp.getMinimumMaximum();
    bool isContinuous = sp.isContinuous(2);
    ::std::cout << "isContinuous = " << isContinuous << ::std::endl;
    ::std::cout << ::std::endl;
  }
  ::std::cout << "Mission Success" << ::std::endl;
  return 0;
}