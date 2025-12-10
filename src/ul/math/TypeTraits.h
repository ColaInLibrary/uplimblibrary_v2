#ifndef UL_MATH_TYPETRAITS_H
#define UL_MATH_TYPETRAITS_H

//#define EIGEN_MATRIXBASE_PLUGIN <rl/math/MatrixBaseAddons.h>
//#define EIGEN_QUATERNIONBASE_PLUGIN <rl/math/QuaternionBaseAddons.h>
//#define EIGEN_TRANSFORM_PLUGIN <rl/math/TransformAddons.h>

#include <Eigen/Core>
#include <algorithm>
#include <iterator>
#include <limits>
#include <ul/std/iterator.h>

namespace ul {
namespace math {
template <typename T, typename Enable = void>
class TypeTraits {
 public:
  typedef typename T::value_type value_type;

  static T Constant(const ::std::size_t& i, const T& value) { return T(i, value); }

  static T Zero(const ::std::size_t& i) { return T(i, 0); }

  static T abs(const T& t) {
    using ::std::abs;
    using ::std::transform;
    using ::ul::std17::size;
    T res(size(t));
    transform(t.begin(), t.end(), res.begin(), static_cast<value_type (*)(value_type)>(&abs));
    return res;
  }

  static bool equal(const T& lhs, const T& rhs, const value_type& epsilon = ::Eigen::NumTraits<value_type>::dummy_precision()) {
    using ::std::abs;
    using ::std::begin;
    using ::std::end;
    using ::std::min;

    auto first1 = begin(lhs);
    auto last1 = end(lhs);
    auto first2 = begin(rhs);

    value_type norm = value_type();
    value_type norm1 = value_type();
    value_type norm2 = value_type();

    while (first1 != last1) {
      value_type tmp = abs(*first1 - *first2);
      norm += tmp * tmp;
      value_type tmp1 = abs(*first1);
      norm1 += tmp1 * tmp1;
      value_type tmp2 = abs(*first2);
      norm2 += tmp2 * tmp2;
      ++first1;
      ++first2;
    }

    return norm <= epsilon * epsilon * min(norm1, norm2);
  }

  static T max_element(const T& t) {
    using ::std::max_element;
    return max_element(t.begin(), t.end());
  }

  static T min_element(const T& t) {
    using ::std::min_element;
    return min_element(t.begin(), t.end());
  }

  static ::std::size_t size(const T& t) {
    using ::ul::std17::size;
    return size(t);
  }

 protected:
 private:
};

template <typename T>
class TypeTraits<T, typename ::std::enable_if<::std::is_integral<T>::value>::type> {
 public:
  typedef T value_type;

  static T Constant(const ::std::size_t& i, const T& value) { return value; }

  static T Zero(const ::std::size_t& i) { return 0; }

  static T abs(const T& t) { return ::std::abs(t); }

  static bool equal(const T& lhs, const T& rhs) { return lhs == rhs; }

  static T max_element(const T& t) { return t; }

  static T min_element(const T& t) { return t; }

  static ::std::size_t size(const T& t) { return 1; }

 protected:
 private:
};

template <typename T>
class TypeTraits<T, typename ::std::enable_if<::std::is_floating_point<T>::value>::type> {
 public:
  typedef T value_type;

  static T Constant(const ::std::size_t& i, const T& value) { return value; }

  static T Zero(const ::std::size_t& i) { return 0; }

  static T abs(const T& t) { return ::std::abs(t); }

  static bool equal(const T& lhs, const T& rhs, const T& epsilon = ::Eigen::NumTraits<T>::dummy_precision()) {
    return ::Eigen::internal::isApprox(lhs, rhs, epsilon);
  }

  static T max_element(const T& t) { return t; }

  static T min_element(const T& t) { return t; }

  static ::std::size_t size(const T& t) { return 1; }

 protected:
 private:
};

template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
class TypeTraits<::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>> {
 public:
  typedef Scalar value_type;

  static ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols> Constant(const ::std::size_t& i, const Scalar& value) {
    return ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>::Constant(i, value);
  }

  static ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols> Zero(const ::std::size_t& i) {
    return ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>::Zero(i);
  }

  static ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols> abs(const ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& t) {
    return t.abs();
  }

  static bool equal(const ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& lhs,
                    const ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& rhs,
                    const Scalar& epsilon = ::Eigen::NumTraits<Scalar>::dummy_precision()) {
    return lhs.isApprox(rhs, epsilon);
  }

  static Scalar max_element(const ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& t) { return t.maxCoeff(); }

  static Scalar min_element(const ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& t) { return t.minCoeff(); }

  static ::std::size_t size(const ::Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& t) { return t.size(); }

 protected:
 private:
};

template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
class TypeTraits<::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>> {
 public:
  typedef Scalar value_type;

  static ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> Constant(const ::std::size_t& i, const Scalar& value) {
    return ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>::Constant(i, value);
  }

  static ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> Zero(const ::std::size_t& i) {
    return ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>::Zero(i);
  }

  static ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> abs(const ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& t) {
    return t.cwiseAbs();
  }

  static bool equal(const ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& lhs,
                    const ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& rhs,
                    const Scalar& epsilon = ::Eigen::NumTraits<Scalar>::dummy_precision()) {
    return lhs.isApprox(rhs, epsilon);
  }

  static Scalar max_element(const ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& t) { return t.maxCoeff(); }

  static Scalar min_element(const ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& t) { return t.minCoeff(); }

  static ::std::size_t size(const ::Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& t) { return t.size(); }

 protected:
 private:
};
}  // namespace math
}  // namespace ul

#endif  // UL_MATH_TYPETRAITS_H
