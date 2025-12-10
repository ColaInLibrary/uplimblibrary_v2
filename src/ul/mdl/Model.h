#ifndef UL_MDL_MODEL_H
#define UL_MDL_MODEL_H

#include <ul/math/Transform.h>

#include <boost/graph/adjacency_list.hpp>
#include <memory>
#include <random>
#include <string>
#include <vector>
// #include <ul/math/Units.h>
#include <ul/math/Vector.h>

//#include "Frame.h"
//#include "Transform.h"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include <pinocchio/multibody/frame.hpp>


namespace ul {

namespace mdl {

class Model {
 public:
  Model(const std::string& urdf_path);

  virtual ~Model();

  bool existFrame(const ::std::string& frame_name);

//  ::ul::math::Vector generatePositionGaussian(const ::ul::math::Vector& mean, const ::ul::math::Vector& sigma);

  ::std::string getBaseFrameName() const;

  ::std::size_t getDof() const;

  ::std::size_t getDofPosition() const;

  ::std::size_t getFrames() const;

  ::std::size_t getFrameId(const ::std::string& frame_name) const;

  ::ul::math::Vector getHomePosition() const;

  ::ul::math::Vector getNeutralPosition() const;

  ::Eigen::VectorXi getIndexURDFInverse() const;

  ::Eigen::VectorXi getIndexURDF() const;

  ::std::size_t getJointIndex(const ::std::string& joint_name) const;

  ::std::size_t getOperationalFrameIndex() const;

  ::std::string getOperationalFrameName() const;

  ::ul::math::Transform getOperationalTransform(const ::std::string &frame_name);

  ::ul::math::Transform getOperationalTransform(const ::std::size_t& frame_id);

  ::ul::math::Transform getWorldMdhBaseTransform(const ::std::size_t& frame_id, const ::ul::math::Vector& q);

  ::ul::math::Vector6 getOperationalVelocity(const ::std::size_t& frame_id);

  ::ul::math::Vector6 getOperationalAcceleration(const ::std::size_t& frame_id);

  const ::std::string& getManufacturer() const;

  ::ul::math::Vector getMaximum() const;

  ::ul::math::Vector getMinimum() const;

  const ::std::string& getName();
  
  const ::std::vector<::std::string> getJointName();

  ::ul::math::Vector getPosition() const;

//  Transform* getTransform(const ::std::size_t& i) const;

  ::std::size_t getTransforms() const;

  ::ul::math::Vector getSpeed() const;

  ::ul::math::Vector getTorque() const;

  ::ul::math::Vector getVelocity() const;

//  World* getWorld() const;

//  const ::ul::math::Vector3& getWorldGravity() const;
  virtual void loadRobot(const std::string& urdf_file);

  void seed(const ::std::mt19937::result_type& value);

  void setAcceleration(const ::ul::math::Vector& qdd);

  void setBaseFrameName(const ::std::string& name);

  void setHomePosition(const ::ul::math::Vector& home);

  void setIndexURDFInverse(const ::Eigen::VectorXi& idx);

  void setIndexURDF(const ::Eigen::VectorXi& idx);

  void setMaximum(const ::ul::math::Vector& max);

  void setMinimum(const ::ul::math::Vector& min);

  void setName(const ::std::string& name);

  void setOperationalFrameIndex(const ::std::size_t& index);

  void setOperationalFrameName(const ::std::string& name);

  void setPosition(const ::ul::math::Vector& q);

  void setSpeed(const ::ul::math::Vector& speed);

  void setTorque(const ::ul::math::Vector& tau);

  virtual void update();


 protected:
  ::std::string base_name;

  ::pinocchio::Data data;

  ::std::size_t dof;

  ::std::size_t dofPosition;

  ::std::size_t ee_index;

  ::std::string ee_name;

  ::ul::math::Vector home;

  ::std::string manufacturer;

  ::pinocchio::Model model;

  ::std::string name;

  ::ul::math::Vector q, qd, qdd, tau;

//  ::std::vector<Transform*> transforms;

 private:
  ::std::uniform_real_distribution<::ul::math::Real>::result_type rand();

  ::std::uniform_real_distribution<::ul::math::Real> randDistribution;

  ::std::mt19937 randEngine;

  ::Eigen::VectorXi urdf_idx, urdf_idx_inv;
};
}  // namespace mdl
}  // namespace ul

#endif  // UL_MDL_MODEL_H
