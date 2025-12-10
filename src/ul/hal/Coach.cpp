#include "Coach.h"

#include <boost/iostreams/stream.hpp>
#include <cassert>
#include <string>
#include <thread>

namespace ul {
namespace hal {
Coach::Coach(const ::std::size_t& dof, const ::ul::math::Real& updateRate)
    : AxisController(dof),
      CyclicDevice(updateRate),
      JointAccelerationActuator(dof),
      JointMode(dof),
      JointModeOperation(dof),
      JointOffsetTorqueActuator(dof),
      JointOffsetVelocityActuator(dof),
      JointPositionActuator(dof),
      JointPositionSensor(dof),
      JointTorqueActuator(dof),
      JointTorqueSensor(dof),
      JointVelocityActuator(dof),
      JointVelocitySensor(dof),
      act_mode(dof, 8),
      act_q(::ul::math::Vector::Zero(dof)),
      act_qd(::ul::math::Vector::Zero(dof)),
      act_qdd(::ul::math::Vector::Zero(dof)),
      act_status_word(dof),
      act_tau(::ul::math::Vector::Zero(dof)),
      act_upload_time(0),
      cmd_control_word(dof),
      cmd_down_time(0),
      cmd_mode_operation(dof),
      cmd_offset_tau(::ul::math::Vector::Zero(dof)),
      cmd_offset_vel(::ul::math::Vector::Zero(dof)),
      cmd_q(::ul::math::Vector::Zero(dof)),
      cmd_q_last(::ul::math::Vector::Zero(dof)),
      cmd_qd(::ul::math::Vector::Zero(dof)),
      cmd_qd_last(::ul::math::Vector::Zero(dof)),
      cmd_qdd(::ul::math::Vector::Zero(dof)),
      cmd_qdd_last(::ul::math::Vector::Zero(dof)),
      cmd_tau(::ul::math::Vector::Zero(dof)) {}

Coach::~Coach() {}

void Coach::close() {}

::std::vector<uint8_t> Coach::getJointMode() const {return this->act_mode;}

::ul::math::Vector Coach::getJointPosition() const {return this->act_q;}

::ul::math::Vector Coach::getJointVelocity() const {return this->act_qd;}

::ul::math::Vector Coach::getJointTorque() const {return this->act_tau;}

void Coach::getJointMode(const ::std::vector<uint8_t>& mode) {
  assert(this->getDof() == mode.size());
  this->act_mode = mode;
}

void Coach::getJointPosition(const ::ul::math::Vector& q) {
  assert(this->getDof() == q.size());
  this->act_q = q;
}

void Coach::getJointStatusWord(const ::std::vector<uint16_t>& status_word) {
  assert(this->getDof() == status_word.size());
  this->act_status_word = status_word;
}

void Coach::getJointTorque(const ::ul::math::Vector& tau) {
  assert(this->getDof() == tau.size());
  this->act_tau = tau;
}

void Coach::getJointUploadTime(const ::ul::math::Real& upload_time) {
  this->act_upload_time = upload_time;
}

void Coach::getJointVelocity(const ::ul::math::Vector& qd) {
  assert(this->getDof() == qd.size());
  this->act_qd = qd;
}

::ul::math::Vector Coach::getLastJointPosition() const {
  return cmd_q_last;
}

::ul::math::Vector Coach::getLastCommandJointPosition() const { return cmd_q_last; }

::ul::math::Vector Coach::getLastCommandJointVelocity() const { return cmd_qd_last; }

::ul::math::Vector Coach::getLastCommandJointAcceleration() const { return cmd_qdd_last; }

void Coach::setJointAcceleration(const ::ul::math::Vector& qdd) {
  assert(this->getDof() == qdd.size());
  this->cmd_qdd = qdd;
}

void Coach::setJointModeOperation(const ::std::vector<uint8_t>& mode_operation) {
  assert(this->getDof() == mode_operation.size());
  this->cmd_mode_operation = mode_operation;
}

void Coach::setJointPosition(const ::ul::math::Vector& q) {
  assert(this->getDof() == q.size());
  this->cmd_q = q;
}

void Coach::setJointOffsetTorqueActuator(const ::ul::math::Vector& offset_tau) {
  assert(this->getDof() == offset_tau.size());
  this->cmd_offset_tau = offset_tau;
}

void Coach::setJointOffsetVelocityActuator(const ::ul::math::Vector& offset_vel) {
  assert(this->getDof() == offset_vel.size());
  this->cmd_offset_vel = offset_vel;
}
void Coach::setJointTorque(const ::ul::math::Vector& tau) {
  assert(this->getDof() >= tau.size());
  this->cmd_tau = tau;
}

void Coach::setJointVelocity(const ::ul::math::Vector& qd) {
  assert(this->getDof() >= qd.size());
  this->cmd_qd = qd;
}

void Coach::setLastJointPosition(const ::ul::math::Vector& q) {
  assert(this->getDof() >= q.size());
  this->cmd_q_last = q;
}

void Coach::setLastJointVelocity(const ::ul::math::Vector& qd) {
  assert(this->getDof() >= qd.size());
  this->cmd_qd_last = qd;
}

void Coach::setLastJointAcceleration(const ::ul::math::Vector& qdd) {
  assert(this->getDof() >= qdd.size());
  this->cmd_qdd_last = qdd;
}

void Coach::start() {
//  this->setRunning(true);
}

void Coach::step() {
//  ::std::chrono::steady_clock::time_point start = ::std::chrono::steady_clock::now();
//
//  this->out << 6 << " " << this->i << ::std::endl;
//
//  this->socket.send(this->out.str().c_str(), this->out.str().length());
//
//  this->out.clear();
//  this->out.str("");
//
//  this->in.fill(0);
//  this->socket.recv(this->in.data(), this->in.size());
//
//  ::std::this_thread::sleep_until(start + this->getUpdateRate());
}

void Coach::stop() {
//  this->out.clear();
//  this->out.str("");
//  this->setRunning(false);
}

::ul::math::Vector Coach::getCommandJointAcceleration() const { return this->cmd_qdd; }

::ul::math::Vector Coach::getCommandJointPosition() const { return this->cmd_q; }

::ul::math::Vector Coach::getCommandJointVelocity() const { return this->cmd_qd;}

::ul::math::Vector Coach::getCommandJointTorque() const { return this->cmd_tau;}

::std::vector<::std::uint16_t> Coach::getCommandJointCtrlwd() const { return this->cmd_control_word; }

::std::vector<::std::uint8_t> Coach::getCommandJointModeOperation() const { return this->cmd_mode_operation; }

::ul::math::Vector Coach::getCommandJointOffsetTorque() const { return this->cmd_offset_tau; }

::ul::math::Vector Coach::getCommandJointOffsetVelocity() const { return this->cmd_offset_vel; }

::ul::math::Real Coach::getCommandDownTime() const { return this->cmd_down_time; }

}  // namespace hal
}  // namespace ul
