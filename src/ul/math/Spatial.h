/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-1-6
 * @Version       : 0.0.1
 * @File          : Spatial.h
 ******************************************************************************
 */

#ifndef UL_SRC_UL_MATH_SPATIAL_H_
#define UL_SRC_UL_MATH_SPATIAL_H_

#define EIGEN_MATRIXBASE_PLUGIN <ul/math/MatrixBaseAddons.h>
// #define EIGEN_QUATERNIONBASE_PLUGIN <rl/math/QuaternionBaseAddons.h>
// #define EIGEN_TRANSFORM_PLUGIN <rl/math/TransformAddons.h>

#include "Real.h"
#include "spatial/ArticulatedBodyInertia.h"
#include "spatial/ArticulatedBodyInertia.hxx"
#include "spatial/ForceVector.h"
#include "spatial/ForceVector.hxx"
#include "spatial/MotionVector.h"
#include "spatial/MotionVector.hxx"
#include "spatial/PlueckerTransform.h"
#include "spatial/PlueckerTransform.hxx"
#include "spatial/RigidBodyInertia.h"
#include "spatial/RigidBodyInertia.hxx"

namespace ul {
namespace math {
typedef spatial::ArticulatedBodyInertia<Real> ArticulatedBodyInertia;

typedef spatial::ForceVector<Real> ForceVector;

typedef spatial::MotionVector<Real> MotionVector;

typedef spatial::PlueckerTransform<Real> PlueckerTransform;

typedef spatial::RigidBodyInertia<Real> RigidBodyInertia;
}  // namespace math
}  // namespace ul

#endif  // UL_SRC_UL_MATH_SPATIAL_H_
