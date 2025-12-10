#ifndef UL_MDL_KINEMATIC_H
#define UL_MDL_KINEMATIC_H

#include <ul/math/Matrix.h>

#include "Model.h"

namespace ul {
namespace mdl {
class Kinematic : public Model {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Kinematic(const std::string& urdf_path);

  virtual ~Kinematic();

  void calculateJacobian(const bool &inWorldFrame = true);

  void calculateJacobian(::ul::math::Matrix &J,
                         const bool &inWorldFrame = true);

  void calculateJacobianDerivative(const bool &inWorldFrame = true);

  void calculateJacobianDerivative(::ul::math::Vector &Jdqd,
                                   const bool &inWorldFrame = true);

  void calculateJacobianInverse(const ::ul::math::Real &lambda = 0,
                                const bool &doSvd = true);

  void calculateJacobianInverse(const ::ul::math::Matrix &J,
                                ::ul::math::Matrix &invJ,
                                const ::ul::math::Real &lambda = 0,
                                const bool &doSvd = true) const;

  void calculateJacobianInverse_user(const ::ul::math::Matrix &J,
                                ::ul::math::Matrix &invJ,
                                const ::ul::math::Real &lambda = 0,
                                const bool &doSvd = true) const;

  ::ul::math::Real calculateManipulabilityMeasure() const;

  ::ul::math::Real
  calculateManipulabilityMeasure(const ::ul::math::Matrix &J) const;

  void forwardAcceleration();

  void forwardPosition();

  void forwardVelocity();

  const ::ul::math::Matrix &getJacobian() const;

  const ::ul::math::Vector &getJacobianDerivative() const;

  const ::ul::math::Matrix &getJacobianInverse() const;

  bool isSingular(const ::ul::math::Real &lambda) const;

  bool isSingular(const ::ul::math::Matrix &J, const ::ul::math::Real &lambda) const;

  virtual void update();

protected:
  ::ul::math::Matrix invJ;

  // 通过 urdf 读进来的机器人模型所对应的 Jacobian 矩阵，可能是 6x7，也可能是 6x14，也可能是 6x16
  ::ul::math::Matrix J;

  // 机器人左右手臂的 Jacobian 矩阵，大小均为 6x7，如果后续扩展为 8 自由度，则需要修改
  ::ul::math::Matrix Jleft, Jright;

  ::ul::math::Vector Jdqd;

private:
};
} // namespace mdl
} // namespace ul

#endif // UL_MDL_KINEMATIC_H
