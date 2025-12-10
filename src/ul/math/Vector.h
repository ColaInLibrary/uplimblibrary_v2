#ifndef UL_MATH_VECTOR_H
#define UL_MATH_VECTOR_H

//#define EIGEN_MATRIXBASE_PLUGIN <rl/math/MatrixBaseAddons.h>
//#define EIGEN_QUATERNIONBASE_PLUGIN <rl/math/QuaternionBaseAddons.h>
//#define EIGEN_TRANSFORM_PLUGIN <rl/math/TransformAddons.h>

#include <Eigen/Core>

#include "Real.h"

namespace ul {
namespace math {
typedef ::Eigen::Matrix<Real, ::Eigen::Dynamic, 1> Vector;

typedef ::Eigen::Matrix<uint8_t, ::Eigen::Dynamic, 1> Vector_u8;

typedef ::Eigen::Matrix<uint16_t, ::Eigen::Dynamic, 1> Vector_u16;

typedef ::Eigen::Matrix<Real, 2, 1> Vector2;

typedef ::Eigen::Matrix<Real, 3, 1> Vector3;

typedef ::Eigen::Matrix<Real, 4, 1> Vector4;

typedef ::Eigen::Matrix<Real, 6, 1> Vector6;

typedef ::Eigen::Matrix<Real, 7, 1> Vector7;

typedef ::Eigen::VectorBlock<Vector, ::Eigen::Dynamic> VectorBlock;

typedef ::Eigen::VectorBlock<Vector2, ::Eigen::Dynamic> Vector2Block;

typedef ::Eigen::VectorBlock<Vector3, ::Eigen::Dynamic> Vector3Block;

typedef ::Eigen::VectorBlock<Vector4, ::Eigen::Dynamic> Vector4Block;

typedef ::Eigen::VectorBlock<Vector6, ::Eigen::Dynamic> Vector6Block;

typedef ::Eigen::VectorBlock<Vector7, ::Eigen::Dynamic> Vector7Block;

typedef ::Eigen::VectorBlock<const Vector, ::Eigen::Dynamic> ConstVectorBlock;

typedef ::Eigen::VectorBlock<const Vector2, ::Eigen::Dynamic> ConstVector2Block;

typedef ::Eigen::VectorBlock<const Vector3, ::Eigen::Dynamic> ConstVector3Block;

typedef ::Eigen::VectorBlock<const Vector4, ::Eigen::Dynamic> ConstVector4Block;

typedef ::Eigen::VectorBlock<const Vector6, ::Eigen::Dynamic> ConstVector6Block;

typedef ::Eigen::VectorBlock<const Vector7, ::Eigen::Dynamic> ConstVector7Block;

typedef ::Eigen::Ref<Vector> VectorRef;

typedef ::Eigen::Ref<const Vector> ConstVectorRef;
}  // namespace math
}  // namespace ul

#endif  // UL_MATH_VECTOR_H
