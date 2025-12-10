#ifndef UL_MATH_ROTATION_H
#define UL_MATH_ROTATION_H

#define EIGEN_MATRIXBASE_PLUGIN <ul/math/MatrixBaseAddons.h>
// #define EIGEN_QUATERNIONBASE_PLUGIN <rl/math/QuaternionBaseAddons.h>
// #define EIGEN_TRANSFORM_PLUGIN <rl/math/TransformAddons.h>

#include <Eigen/Geometry>

#include "Matrix.h"

namespace ul {
namespace math {
typedef ::Eigen::AngleAxis<Real> AngleAxis;
}
}  // namespace ul

#endif  // UL_MATH_ROTATION_H
