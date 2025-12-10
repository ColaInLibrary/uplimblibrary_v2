// #include "Body.h"
// #include "Exception.h"
#include "Model.h"

//#include "Joint.h"
//#include "World.h"

namespace ul {
namespace mdl {
Model::Model(const std::string& urdf_path)
    : dof(),
      dofPosition(),
      home(),
      name(),
//      transforms(),
      randDistribution(0, 1),
      randEngine(::std::random_device()()) {
  this->loadRobot(urdf_path);
}

Model::~Model() {}

bool Model::existFrame(const ::std::string& frame_name) {
  return this->model.existFrame(frame_name);
}

//::ul::math::Vector Model::generatePositionGaussian(const ::ul::math::Vector& mean, const ::ul::math::Vector& sigma) {
//  ::ul::math::Vector rand(this->getDof());
//
//  for (::std::size_t i = 0; i < this->getDof(); ++i) {
//    rand(i) = this->rand();
//  }
//
//  return this->generatePositionGaussian(rand, mean, sigma);
//}


::std::string Model::getBaseFrameName() const { return this->base_name; }

::std::size_t Model::getDof() const { return this->dof; }

::std::size_t Model::getDofPosition() const { return this->dofPosition; }

::std::size_t Model::getFrames() const { return this->model.frames.size(); }

::std::size_t Model::getFrameId(const ::std::string& frame_name) const {
  assert(this->model.existFrame(frame_name));
  return this->model.getFrameId(frame_name);
}

::ul::math::Vector Model::getHomePosition() const { return this->home(this->urdf_idx_inv); }

::ul::math::Vector Model::getNeutralPosition() const { return ::pinocchio::neutral(model); }

::Eigen::VectorXi Model::getIndexURDFInverse() const { return this->urdf_idx_inv; };

::Eigen::VectorXi Model::getIndexURDF() const { return this->urdf_idx; };

::std::size_t Model::getJointIndex(const ::std::string& joint_name) const { return this->model.getJointId(joint_name); }

::std::size_t Model::getOperationalFrameIndex() const {return this->ee_index; }

::std::string Model::getOperationalFrameName() const { return this->ee_name;}

::ul::math::Transform Model::getOperationalTransform(const ::std::string& frame_name) {
  ::ul::math::Transform T;
  T.matrix().setIdentity();
  ::pinocchio::forwardKinematics(this->model, this->data, this->q);
  ::pinocchio::updateFramePlacements(this->model, this->data);
  setOperationalFrameName(frame_name);
  auto frameId = this->model.getFrameId(ee_name);
  T.matrix().block(0,0,3,3) = this->data.oMf[frameId].rotation();
  T.translation() = this->data.oMf[frameId].translation();
  return T;
}

::ul::math::Transform Model::getOperationalTransform(const ::std::size_t& frame_id) {
  ::ul::math::Transform T;
  T.matrix().setIdentity();
  ::pinocchio::forwardKinematics(this->model, this->data, this->q);
  ::pinocchio::updateFramePlacements(this->model, this->data);
  T.matrix().block(0,0,3,3) = this->data.oMf[frame_id].rotation();
  T.translation() = this->data.oMf[frame_id].translation();
  return T;
}

::ul::math::Transform Model::getWorldMdhBaseTransform(const ::std::size_t& frame_id, const ::ul::math::Vector& q) {
  assert(q.size() == this->getDofPosition());
  ::ul::math::Transform T;
  T.matrix().setIdentity();
  ::ul::math::Vector q_temp = q;
  q_temp.head(14) = getNeutralPosition()(this->urdf_idx_inv).head(14);
  ::ul::math::Vector q_temp2 = q_temp(this->getIndexURDF());
  ::pinocchio::forwardKinematics(this->model, this->data, q_temp2);
  ::pinocchio::updateFramePlacements(this->model, this->data);
  T.matrix().block(0,0,3,3) = this->data.oMf[frame_id].rotation();
  T.translation() = this->data.oMf[frame_id].translation();
  return T;
}

::ul::math::Vector6 Model::getOperationalVelocity(const ::std::size_t& frame_id) {
  ::pinocchio::forwardKinematics(this->model, this->data, this->q, this->qd);
  auto ee_vel = ::pinocchio::getFrameVelocity(this->model, this->data, frame_id, ::pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
  ::ul::math::Vector6 ee_vel_6d = ::ul::math::Vector6::Zero();
  ee_vel_6d.head(3) = ee_vel.linear();
  ee_vel_6d.tail(3) = ee_vel.angular();
  return ee_vel_6d;
}

::ul::math::Vector6 Model::getOperationalAcceleration(const ::std::size_t& frame_id) {
  ::pinocchio::forwardKinematics(this->model, this->data, this->q, this->qd, this->qdd);
  ::pinocchio::updateFramePlacements(this->model, this->data);
  auto ee_acc = getFrameAcceleration(this->model, this->data, frame_id, ::pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
  ::ul::math::Vector6 ee_acc_6d = ::ul::math::Vector6::Zero();
  ee_acc_6d.head(3) = ee_acc.linear();
  ee_acc_6d.tail(3) = ee_acc.angular();
  return ee_acc_6d;
}

const ::std::string& Model::getManufacturer() const { return this->manufacturer; }

::ul::math::Vector Model::getMaximum() const {
  ::ul::math::Vector max(this->getDofPosition());

//  for (::std::size_t i = 0, j = 0; i < this->joints.size(); j += this->joints[i]->getDofPosition(), ++i) {
//    max.segment(j, this->joints[i]->getDofPosition()) = this->joints[i]->getMaximum();
//  }

  return max(this->urdf_idx_inv);
}

::ul::math::Vector Model::getMinimum() const {
  ::ul::math::Vector min(this->getDofPosition());

//  for (::std::size_t i = 0, j = 0; i < this->joints.size(); j += this->joints[i]->getDofPosition(), ++i) {
//    min.segment(j, this->joints[i]->getDofPosition()) = this->joints[i]->getMinimum();
//  }

  return min(this->urdf_idx_inv);
}

const ::std::string& Model::getName() {
  this->name = this->model.name;
  return this->name;
}

const ::std::vector<::std::string> Model::getJointName() {
  ::std::vector<::std::string> name_list;
  for (::pinocchio::JointIndex joint_id = 1; joint_id <= this->dof; ++joint_id) {
    name_list.push_back(this->model.names[joint_id]);
  }
  return name_list;
}

::ul::math::Vector Model::getPosition() const {
//  ::std::cout << "urdf position: \n" << this->q.transpose() << std::endl;
//  ::std::cout << "driver/algorithm position: \n" << this->q(this->urdf_idx_inv).transpose() << std::endl;
  return this->q(this->urdf_idx_inv);
}

::ul::math::Vector Model::getSpeed() const {
  ::ul::math::Vector speed(this->getDof());

//  for (::std::size_t i = 0, j = 0; i < this->joints.size(); j += this->joints[i]->getDof(), ++i) {
//    speed.segment(j, this->joints[i]->getDof()) = this->joints[i]->getSpeed();
//  }

  return speed((this->urdf_idx_inv));
}

::ul::math::Vector Model::getTorque() const {
  ::ul::math::Vector tau(this->getDof());

//  for (::std::size_t i = 0, j = 0; i < this->joints.size(); j += this->joints[i]->getDof(), ++i) {
//    tau.segment(j, this->joints[i]->getDof()) = this->joints[i]->getTorque();
//  }

  return tau((this->urdf_idx_inv));
}

//Transform* Model::getTransform(const ::std::size_t& i) const {
//  assert(i < this->transforms.size());
//
//  return this->transforms[i];
//}

::std::size_t Model::getTransforms() const {
//  return this->transforms.size();
  return 6;
}

::ul::math::Vector Model::getVelocity() const {
  ::ul::math::Vector qd(this->getDof());

//  for (::std::size_t i = 0, j = 0; i < this->joints.size(); j += this->joints[i]->getDof(), ++i) {
//    qd.segment(j, this->joints[i]->getDof()) = this->joints[i]->getVelocity();
//  }
//
//  return this->invGammaVelocity * qd;
  return qd(this->urdf_idx_inv);
}

//World* Model::getWorld() const { return dynamic_cast<World*>(this->tree[this->root].get()); }

//const ::ul::math::Vector3& Model::getWorldGravity() const { return dynamic_cast<World*>(this->tree[this->root].get())->getGravity(); }

void Model::loadRobot(const std::string& urdf_file) {
  ::pinocchio::urdf::buildModel(urdf_file, this->model);
  data = ::pinocchio::Data(this->model);
  this->dof = this->model.nq;
  this->dofPosition = this->model.nq;
  this->q.resize(this->dof);
  q = ::pinocchio::neutral(this->model);
}
//		bool
//		Model::isColliding(const ::std::size_t& i) const
//		{
//			assert(i < this->bodies.size());
//
//			return this->bodies[i]->getCollision();
//		}

::std::uniform_real_distribution<::ul::math::Real>::result_type Model::rand() { return this->randDistribution(this->randEngine); }

void Model::seed(const ::std::mt19937::result_type& value) { this->randEngine.seed(value); }

void Model::setAcceleration(const ::ul::math::Vector& ydd) {
  assert(ydd.size() == this->getDof());
  this->qdd = ydd(this->getIndexURDF());
}

void Model::setBaseFrameName(const ::std::string &name){ this->base_name = name; }
void Model::setHomePosition(const ::ul::math::Vector& home) { this->home = home; }

void Model::setIndexURDFInverse(const ::Eigen::VectorXi& idx) { this->urdf_idx_inv = idx; }

void Model::setIndexURDF(const ::Eigen::VectorXi& idx) { 
  this->urdf_idx = idx;
  this->urdf_idx_inv = ::Eigen::VectorXi::Zero(this->urdf_idx.size());
  for (int i = 0; i < this->urdf_idx.size(); ++i) {
    this->urdf_idx_inv[urdf_idx[i]] = i;
  }
//  std::cout << "urdf_idx: " << urdf_idx.transpose() << std::endl;
//  std::cout << "urdf_idx_inv: " << urdf_idx_inv.transpose() << std::endl;
}

void Model::setMaximum(const ::ul::math::Vector& max) {
//  for (::std::size_t i = 0, j = 0; i < this->joints.size(); j += this->joints[i]->getDofPosition(), ++i) {
//    this->joints[i]->setMaximum(max.segment(j, this->joints[i]->getDofPosition()));
//  }
}

void Model::setMinimum(const ::ul::math::Vector& min) {
//  for (::std::size_t i = 0, j = 0; i < this->joints.size(); j += this->joints[i]->getDofPosition(), ++i) {
//    this->joints[i]->setMinimum(min.segment(j, this->joints[i]->getDofPosition()));
//  }
}

void Model::setName(const ::std::string& name) { this->name = name; }

//		void
//		Model::setOperationalVelocity(const ::std::size_t& i, const ::ul::math::MotionVector& v) const
//		{
//			this->tree[this->leaves[i]]->v = v;
//		}

void Model::setOperationalFrameIndex(const ::std::size_t& index) {
  this->ee_index = index;
}
void Model::setOperationalFrameName(const std::string& name) {
  assert(this->model.existFrame(name));
  this->ee_name = name;
}
void Model::setPosition(const ::ul::math::Vector& y) {
  assert(y.size() == this->getDofPosition());
  this->q = y(this->getIndexURDF());
}

void Model::setSpeed(const ::ul::math::Vector& speed) {
  assert(speed.size() == this->getDof());
  this->qd = speed(this->getIndexURDF());
}

void Model::setTorque(const ::ul::math::Vector& tau) {
//  for (::std::size_t i = 0, j = 0; i < this->joints.size(); j += this->joints[i]->getDof(), ++i) {
//    this->joints[i]->setTorque(tau.segment(j, this->joints[i]->getDof()));
//  }
}

//		::ul::math::Transform&
//		Model::tool(const ::std::size_t& i)
//		{
//			assert(i < this->tools.size());
//
//			return this->tree[this->tools[i]]->x.transform();
//		}

//		const ::ul::math::Transform&
//		Model::tool(const ::std::size_t& i) const
//		{
//			assert(i < this->tools.size());
//
//			return this->tree[this->tools[i]]->x.transform();
//		}

void Model::update() {
//  this->bodies.clear();
//  this->dof = 0;
//  this->dofPosition = 0;
//  this->elements.clear();
//  this->joints.clear();
//  this->leaves.clear();
//  this->tools.clear();
//  this->transforms.clear();
//
//  this->update(this->root);
//
//  for (::std::size_t i = 0; i < this->joints.size(); ++i) {
//    this->dof += this->joints[i]->getDof();
//    this->dofPosition += this->joints[i]->getDofPosition();
//  }
//
//  this->gammaPosition = ::ul::math::Matrix::Identity(this->getDofPosition(), this->getDofPosition());
//  this->gammaVelocity = ::ul::math::Matrix::Identity(this->getDof(), this->getDof());
//  this->home = ::ul::math::Vector::Zero(this->getDofPosition());
//  this->invGammaPosition = ::ul::math::Matrix::Identity(this->getDofPosition(), this->getDofPosition());
//  this->invGammaVelocity = ::ul::math::Matrix::Identity(this->getDof(), this->getDof());
//}
//
//void Model::update(const Vertex& u) {
//  Frame* frame = this->tree[u].get();
//  this->elements.push_back(frame);
//  this->frames.push_back(frame);
//
//  //			if (Body* body = dynamic_cast<Body*>(frame))
//  //			{
//  //				this->bodies.push_back(body);
//  //			}
//
//  if (::boost::out_degree(u, this->tree) > 0) {
//    for (OutEdgeIteratorPair i = ::boost::out_edges(u, this->tree); i.first != i.second; ++i.first) {
//      Edge e = *i.first;
//      Vertex v = ::boost::target(e, this->tree);
//
//      Transform* transform = this->tree[e].get();
//      this->elements.push_back(transform);
//      this->transforms.push_back(transform);
//      transform->in = this->tree[u].get();
//      transform->out = this->tree[v].get();
//
//      if (Joint* joint = dynamic_cast<Joint*>(transform)) {
//        this->joints.push_back(joint);
//      }
//
//      this->update(v);
//    }
//  } else {
//    this->leaves.push_back(u);
//
//    for (InEdgeIteratorPair i = ::boost::in_edges(u, this->tree); i.first != i.second; ++i.first) {
//      this->tools.push_back(*i.first);
//    }
//  }
}

//		::ul::math::Transform&
//		Model::world()
//		{
//			return this->tree[this->root]->x.transform();
//		}

//		const ::ul::math::Transform&
//		Model::world() const
//		{
//			return this->tree[this->root]->x.transform();
//		}
}  // namespace mdl
}  // namespace ul
