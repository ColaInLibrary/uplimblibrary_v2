#ifndef UL_HAL_COACH_H
#define UL_HAL_COACH_H

#include <array>
#include <sstream>
#include <string>

#include "CyclicDevice.h"
#include "JointAccelerationActuator.h"
#include "JointMode.h"
#include "JointModeOperation.h"
#include "JointOffsetTorqueActuator.h"
#include "JointOffsetVelocityActuator.h"
#include "JointPositionActuator.h"
#include "JointPositionSensor.h"
#include "JointTorqueActuator.h"
#include "JointTorqueSensor.h"
#include "JointVelocityActuator.h"
#include "JointVelocitySensor.h"

namespace ul {
namespace hal {
class Coach : public CyclicDevice,
              public JointAccelerationActuator,
              public JointMode,
              public JointModeOperation,
              public JointOffsetTorqueActuator,
              public JointOffsetVelocityActuator,
              public JointPositionActuator,
              public JointPositionSensor,
              public JointTorqueActuator,
              public JointTorqueSensor,
              public JointVelocityActuator,
              public JointVelocitySensor {
 public:
  Coach(const ::std::size_t& dof, const ::ul::math::Real& updateRate);

  virtual ~Coach();

  void close();

  // 获取下发到驱动器的数据
  ::ul::math::Vector getCommandJointAcceleration() const;

  ::ul::math::Vector getCommandJointPosition() const;

  ::ul::math::Vector getCommandJointVelocity() const;

  ::ul::math::Vector getCommandJointTorque() const;

  ::std::vector<::std::uint16_t> getCommandJointCtrlwd() const;

  ::std::vector<::std::uint8_t> getCommandJointModeOperation() const;

  ::ul::math::Vector getCommandJointOffsetTorque() const;

  ::ul::math::Vector getCommandJointOffsetVelocity() const;

  ::ul::math::Real getCommandDownTime() const;

  // Controller 获取实际的关节数据，直接访问对应的 act_*_ 的数据成员
  ::std::vector<uint8_t> getJointMode() const override;

  ::ul::math::Vector getJointPosition() const override;

  ::ul::math::Vector getJointTorque() const override;

  ::ul::math::Vector getJointVelocity() const override;

  // 获取从驱动器读进来的数据，将其设置到对应的数据成员中
  void getJointMode(const ::std::vector<uint8_t>& mode);

  void getJointPosition(const ::ul::math::Vector& q);

  void getJointStatusWord(const ::std::vector<uint16_t>& status_word);

  void getJointTorque(const ::ul::math::Vector& tau);

  void getJointUploadTime(const ::ul::math::Real&  upload_time);

  void getJointVelocity(const ::ul::math::Vector& qd);

  ::ul::math::Vector getLastJointPosition() const override;

  ::ul::math::Vector getLastCommandJointPosition() const;

  ::ul::math::Vector getLastCommandJointVelocity() const;

  ::ul::math::Vector getLastCommandJointAcceleration() const;

  // 设置下发到驱动器的数据
  void setJointAcceleration(const ::ul::math::Vector& qdd) override;

  void setJointModeOperation(const ::std::vector<uint8_t>& mode_operation) override;

  void setJointOffsetTorqueActuator(const ::ul::math::Vector& offset_tau) override;

  void setJointOffsetVelocityActuator(const ::ul::math::Vector& offset_vel) override;

  void setJointPosition(const ::ul::math::Vector& q) override;

  void setJointTorque(const ::ul::math::Vector& tau) override;

  void setJointVelocity(const ::ul::math::Vector& qd) override;

  void setLastJointPosition(const ::ul::math::Vector& q) override;

  void setLastJointVelocity(const ::ul::math::Vector& qd) override;

  void setLastJointAcceleration(const ::ul::math::Vector& qdd) override;

  void start();

  void step();

  void stop();

 protected:
  // 从驱动器读进来的数据，对应结构体 UPLIMB_INPUT_INFO_STRUCT 中的数据内容和格式
  ::std::vector<::std::uint8_t> act_mode;

  ::ul::math::Vector act_q;

  ::ul::math::Vector act_qd;

  ::ul::math::Vector act_qdd;

  ::std::vector<::std::uint16_t> act_status_word;

  ::ul::math::Vector act_tau;

  ::ul::math::Real act_upload_time;

  // 下发到驱动器的数据，对应结构体 UPLIMB_OUTPUT_INFO_STRUCT  中的数据内容和格式
  ::std::vector<::std::uint16_t> cmd_control_word;

  ::ul::math::Real cmd_down_time;

  ::std::vector<::std::uint8_t> cmd_mode_operation;

  ::ul::math::Vector cmd_offset_tau;

  ::ul::math::Vector cmd_offset_vel;

  ::ul::math::Vector cmd_q, cmd_q_last;

  ::ul::math::Vector cmd_qd, cmd_qd_last;

  ::ul::math::Vector cmd_qdd, cmd_qdd_last;

  ::ul::math::Vector cmd_qddd;

  ::ul::math::Vector cmd_tau;
 private:

};
}  // namespace hal
}  // namespace ul

#endif  // UL_HAL_COACH_H
