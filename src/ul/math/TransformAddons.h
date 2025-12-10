#ifndef UL_MATH_TRANSFORMADDONS_H
#define UL_MATH_TRANSFORMADDONS_H

#ifdef DOXYGEN_SHOULD_PARSE_THIS
namespace Eigen {
template <typename _Scalar, int _Dim, int _Mode, int _Options>
class Transform {
#endif

  template <typename OtherScalar, int OtherDim, int OtherMode, int OtherOptions>
  inline Scalar distance(const Transform<OtherScalar, OtherDim, OtherMode, OtherOptions>& other, const Scalar& weight = 1) const {
    using ::std::pow;
    using ::std::sqrt;

    Quaternion<Scalar> q1(rotation());
    Quaternion<Scalar> q2(other.rotation());

    return sqrt(pow(other(0, 3) - (*this)(0, 3), 2) + pow(other(1, 3) - (*this)(1, 3), 2) + pow(other(2, 3) - (*this)(2, 3), 2) +
                weight * pow(q1.angularDistance(q2), 2));
  }

  template <typename OtherScalar1, typename OtherScalar2, typename OtherScalar3, typename OtherScalar4>
  inline void fromDenavitHartenbergPaul(const OtherScalar1& d, const OtherScalar2& theta, const OtherScalar3& a, const OtherScalar4& alpha) {
    using ::std::cos;
    using ::std::sin;

    Scalar cosAlpha = cos(alpha);
    Scalar cosTheta = cos(theta);
    Scalar sinAlpha = sin(alpha);
    Scalar sinTheta = sin(theta);

    (*this)(0, 0) = cosTheta;
    (*this)(1, 0) = sinTheta;
    (*this)(2, 0) = Scalar(0);
    (*this)(3, 0) = Scalar(0);
    (*this)(0, 1) = -cosAlpha * sinTheta;
    (*this)(1, 1) = cosAlpha * cosTheta;
    (*this)(2, 1) = sinAlpha;
    (*this)(3, 1) = Scalar(0);
    (*this)(0, 2) = sinAlpha * sinTheta;
    (*this)(1, 2) = -sinAlpha * cosTheta;
    (*this)(2, 2) = cosAlpha;
    (*this)(3, 2) = Scalar(0);
    (*this)(0, 3) = a * cosTheta;
    (*this)(1, 3) = a * sinTheta;
    (*this)(2, 3) = d;
    (*this)(3, 3) = Scalar(1);
  }

  template <typename OtherScalar1, typename OtherScalar2, typename OtherScalar3, typename OtherScalar4>
  inline void fromMDH(const OtherScalar1& d, const OtherScalar2& theta, const OtherScalar3& a, const OtherScalar4& alpha) {
    using ::std::cos;
    using ::std::sin;

    Scalar cosAlpha = cos(alpha);
    Scalar cosTheta = cos(theta);
    Scalar sinAlpha = sin(alpha);
    Scalar sinTheta = sin(theta);

    (*this)(0, 0) = cosTheta;
    (*this)(1, 0) = cosAlpha * sinTheta;
    (*this)(2, 0) = sinAlpha * sinTheta;
    (*this)(3, 0) = Scalar(0);
    (*this)(0, 1) = -sinTheta;
    (*this)(1, 1) = cosAlpha * cosTheta;
    (*this)(2, 1) = sinAlpha * cosTheta;
    (*this)(3, 1) = Scalar(0);
    (*this)(0, 2) = Scalar(0);
    (*this)(1, 2) = -sinAlpha;
    (*this)(2, 2) = cosAlpha;
    (*this)(3, 2) = Scalar(0);
    (*this)(0, 3) = a;
    (*this)(1, 3) = -d * sinAlpha;
    (*this)(2, 3) = d * cosAlpha;
    (*this)(3, 3) = Scalar(1);
  }

  inline Matrix<Scalar, 6, 1> getDelta() const {
    Matrix<Scalar, 6, 1> res;

    res(0) = (*this)(0, 3);
    res(1) = (*this)(1, 3);
    res(2) = (*this)(2, 3);
    res(3) = ((*this)(2, 1) - (*this)(1, 2)) * Scalar(0.5);
    res(4) = ((*this)(0, 2) - (*this)(2, 0)) * Scalar(0.5);
    res(5) = ((*this)(1, 0) - (*this)(0, 1)) * Scalar(0.5);

    return res;
  }

  template <typename OtherScalar>
  inline void setDelta(const Matrix<OtherScalar, 6, 1>& delta) {
    (*this)(0, 0) = Scalar(0);
    (*this)(0, 1) = -delta(5);
    (*this)(0, 2) = delta(4);
    (*this)(0, 3) = delta(0);
    (*this)(1, 0) = delta(5);
    (*this)(1, 1) = Scalar(0);
    (*this)(1, 2) = -delta(3);
    (*this)(1, 3) = delta(1);
    (*this)(2, 0) = -delta(4);
    (*this)(2, 1) = delta(3);
    (*this)(2, 2) = Scalar(0);
    (*this)(2, 3) = delta(2);
    (*this)(3, 0) = Scalar(0);
    (*this)(3, 1) = Scalar(0);
    (*this)(3, 2) = Scalar(0);
    (*this)(3, 3) = Scalar(1);
  }

  template <typename OtherScalar1, typename OtherScalar2, typename OtherScalar3, typename OtherScalar4>
  inline void toDenavitHartenbergPaul(OtherScalar1& d, OtherScalar2& theta, OtherScalar3& a, OtherScalar4& alpha) const {
    using ::std::abs;
    using ::std::atan2;

    assert(abs((*this)(2, 0)) <= ::std::numeric_limits<Scalar>::epsilon());

    d = (*this)(2, 3);
    theta = atan2((*this)(1, 0), (*this)(0, 0));

    Scalar tmp = (*this)(0, 0) + (*this)(1, 0);

    if (abs(tmp) > 0) {
      a = ((*this)(0, 3) + (*this)(1, 3)) / tmp;
    } else if (abs((*this)(1, 0)) > 0) {
      a = (*this)(1, 3) / (*this)(1, 0);
    } else if (abs((*this)(0, 0)) > 0) {
      a = (*this)(0, 3) / (*this)(0, 0);
    } else {
      a = ::std::numeric_limits<Scalar>::quiet_NaN();
    }

    alpha = atan2((*this)(2, 1), (*this)(2, 2));
  }

#ifdef DOXYGEN_SHOULD_PARSE_THIS
}
}
#endif

#ifdef UL_EIGEN_TRANSFORM_PLUGIN
#include UL_EIGEN_TRANSFORM_PLUGIN
#endif

#endif  // UL_MATH_TRANSFORMADDONS_H
