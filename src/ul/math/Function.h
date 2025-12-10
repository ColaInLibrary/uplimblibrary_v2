#ifndef UL_MATH_FUNCTION_H
#define UL_MATH_FUNCTION_H

#include "Real.h"

namespace ul {
namespace math {
/**
 * A mathematical mapping from Real -> ArrayX.
 *
 * A Function is guaranteed to be defined in the interval [lower() upper()],
 * and may be defined outside this interval. Its computation is expected
 * to be numerically stable, accurate and efficient.
 */
template <typename T> class Function {
public:
  typedef T value_type;

  Function(const Real &x0 = 0, const Real &x1 = 1) : x0(x0), x1(x1) {}

  virtual ~Function() {}

  virtual Function *clone() const = 0;

  Real duration() const { return this->upper() - this->lower(); }

  Real &lower() { return this->x0; }

  const Real &lower() const { return this->x0; }

  Real &upper() { return this->x1; }

  const Real &upper() const { return this->x1; }

  /**
   * Evaluates the function or a derivative thereof for a given value x.
   *
   * Some functions are only defined in the interval [lower(), upper()],
   * and fail to evaluate outside of
   * [lower() - functionBoundary, upper() + functionBoundary].
   * In Debug mode, this is signaled by failing asserts.
   * In Release mode, the function is evaluated if algebraically possible,
   * or will return an empty ArrayX otherwise.
   * Some functions are not indefinitely often differentiable,
   * and will return a NaN array for all higher orders.
   *
   * @param[in] x Input value of the function or derivative
   * @param[in] derivative Order of the derivative to be evaluated
   */
  virtual T operator()(const Real &x,
                       const ::std::size_t &derivative = 0) const = 0;

protected:
  static constexpr Real functionBoundary = static_cast<Real>(1.0e-8);

  Real x0;

  Real x1;

private:
};
} // namespace math
} // namespace ul

#endif // UL_MATH_FUNCTION_H
