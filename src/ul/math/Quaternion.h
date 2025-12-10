#ifndef UL_MATH_QUATERNION_H
#define UL_MATH_QUATERNION_H

#define EIGEN_MATRIXBASE_PLUGIN <ul/math/MatrixBaseAddons.h>
#define EIGEN_QUATERNIONBASE_PLUGIN <ul/math/QuaternionBaseAddons.h>
#define EIGEN_TRANSFORM_PLUGIN <ul/math/TransformAddons.h>

#include <Eigen/Geometry>

#include "Real.h"

namespace ul {
namespace math {
typedef ::Eigen::Quaternion<Real> Quaternion;
}
}  // namespace ul

#endif  // UL_MATH_QUATERNION_H
