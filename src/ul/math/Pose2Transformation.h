/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-2-11
 * @Version       : 0.0.1
 * @File          : Pose2Transformation.h
 ******************************************************************************
 */

#ifndef UL_SRC_UL_MATH_POSE2TRANSFORMATION_H_
#define UL_SRC_UL_MATH_POSE2TRANSFORMATION_H_

#include "Vector.h"
#include "Matrix.h"
#include "Transform.h"
#include "Eigen/Eigen"
namespace ul {
namespace math {
inline void Pose2Transformation(const ::ul::math::Vector &pose, ::ul::math::Transform &T) {
  T.setIdentity();
  Matrix33 R;
  R = ::Eigen::AngleAxisd(pose[3], ::Eigen::Vector3d::UnitZ()) *
      ::Eigen::AngleAxisd(pose[4], ::Eigen::Vector3d::UnitY()) *
      ::Eigen::AngleAxisd(pose[5], ::Eigen::Vector3d::UnitX());
  T.rotate(R);
  T.translation() = pose.block<3, 1>(0, 0);
}

inline void Pose2Rotation(const ::ul::math::Vector &pose, ::ul::math::Matrix33 &R) {
  R.setIdentity();
  R = ::Eigen::AngleAxisd(pose[3], ::Eigen::Vector3d::UnitZ()) *
      ::Eigen::AngleAxisd(pose[4], ::Eigen::Vector3d::UnitY()) *
      ::Eigen::AngleAxisd(pose[5], ::Eigen::Vector3d::UnitX());
}

inline void Transform2Pose(const ::ul::math::Transform &T, ::ul::math::Vector6 &pose) {
  pose.segment<3>(0) = T.translation();
  ::ul::math::Vector3 rpy = T.rotation().eulerAngles(2, 1, 0);  // ZYX
  pose.segment<3>(3) = rpy;
}
}  // namespace math
}  // namespace ul
#endif  // UL_SRC_UL_MATH_POSE2TRANSFORMATION_H_
