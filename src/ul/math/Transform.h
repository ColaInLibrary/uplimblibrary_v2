#ifndef UL_MATH_TRANSFORM_H
#define UL_MATH_TRANSFORM_H

 #define EIGEN_MATRIXBASE_PLUGIN <ul/math/MatrixBaseAddons.h>
// #define EIGEN_QUATERNIONBASE_PLUGIN <ul/math/QuaternionBaseAddons.h>
 #define EIGEN_TRANSFORM_PLUGIN <ul/math/TransformAddons.h>

#include <Eigen/Geometry>

#include "Matrix.h"
#include "Quaternion.h"
#include "Rotation.h"
#include "Vector.h"

namespace ul {
namespace math {
/** Rigid transformation in 3D. */
typedef ::Eigen::Transform<Real, 3, ::Eigen::Affine> Transform;

/** Translation in 3D. */
typedef ::Eigen::Translation<Real, 3> Translation;
}  // namespace math
}  // namespace ul

#endif  // UL_MATH_TRANSFORM_H
