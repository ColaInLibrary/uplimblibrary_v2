/**
 ******************************************************************************
 * @Description   : 实现单自由度傅里叶级数的功能
 * @author        : AN Hao
 * @Date          : 25-2-18
 * @Version       : 0.0.1
 * @File          : FourierSeries.h
 ******************************************************************************
 */

#ifndef UL_SRC_UL_MATH_FOURIERSERIES_H_
#define UL_SRC_UL_MATH_FOURIERSERIES_H_
#include "Real.h"
#include "Vector.h"

namespace ul {
namespace math {
class FourierSeries {
  public:
   FourierSeries(double fundamental_frequency, const ::std::vector<Real>& a_coeffs, const std::vector<Real>& b_coeffs)
       : omega(2 * M_PI * fundamental_frequency), a(a_coeffs), b(b_coeffs) {}

   // 计算傅里叶级数的值
   Real operator()(Real t) const {
     Real sum = a[0] / 2; // a_0 / 2 部分
     for (size_t n = 1; n < a.size(); ++n) {
       sum += a[n] * ::std::cos(n * omega * t) + b[n] * ::std::sin(n * omega * t);
     }
     return sum;
   }
  private:
    ::std::vector<Real> a;
    ::std::vector<Real> b;
    Real omega;
};
}
}

#endif  // UL_SRC_UL_MATH_FOURIERSERIES_H_
