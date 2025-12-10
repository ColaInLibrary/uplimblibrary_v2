#ifndef UL_MDL_DYNAMIC_H
#define UL_MDL_DYNAMIC_H

#include "Kinematic.h"

namespace ul {
namespace mdl {
class Dynamic : public Kinematic {
 public:
  Dynamic(const std::string& urdf_path);

  virtual ~Dynamic();

  void calculateCentrifugalCoriolis();

  void calculateCentrifugalCoriolis(::ul::math::Vector& V);

  void calculateGravity();

  void calculateGravity(::ul::math::Vector& G);

  void calculateMassMatrix();

  void calculateMassMatrix(::ul::math::Matrix& M);
  
  // 需要先调用 calculateMassMatrix
  void calculateMassMatrixInverse();

  // 需要先调用 calculateMassMatrix
  void calculateMassMatrixInverse(::ul::math::Matrix& invM);

  void calculateOperationalMassMatrixInverse();

  void calculateOperationalMassMatrixInverse(const ::ul::math::Matrix& J, const ::ul::math::Matrix& invM, ::ul::math::Matrix& invMx) const;

  void calculateFriction();

  void forwardDynamics();

  void setLoadParas(::std::vector<::ul::math::Real>& load_mass, ::std::vector<::ul::math::Vector3>& load_barycenter_trans, ::std::vector<::std::size_t>& ee_id);

  ::ul::math::Vector getCentrifugalCoriolis();

  ::ul::math::Vector getGravity();

  ::ul::math::Vector getFriction();

  const ::ul::math::Matrix& getMassMatrixInverse() const;

  const ::ul::math::Matrix& getMassMatrix() const;

  const ::ul::math::Matrix& getOperationalMassMatrixInverse() const;

  void inverseDynamics();

  void setJointFrictionParas(const double s_k, const ::ul::math::Vector& f_a, const ::ul::math::Vector& f_b, const ::ul::math::Vector& f_c);

  virtual void update();

 protected:
  ::ul::math::Vector G;

  ::ul::math::Matrix invM;

  ::ul::math::Matrix invMx;

  ::ul::math::Matrix M;

  ::ul::math::Vector V;

  ::ul::math::Vector F;

 private:
 double s_k; // 
 
 ::ul::math::Vector f_a, f_b, f_c;  // 关节摩擦模型参数 fric = f_a*sat(qd,s_k)+f_b*qd+f_c

 bool first = true;
 ::std::vector<::pinocchio::Inertia> ee_inertia;
};
}  // namespace mdl
}  // namespace ul

#endif  // UL_MDL_DYNAMIC_H
