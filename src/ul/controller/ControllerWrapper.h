/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-5-21
 * @Version       : 0.0.1
 * @File          : ControllerWrapper.h
 ******************************************************************************
 */

#ifndef UL_SRC_UL_PACK_CONTROLLERWRAPPER_H_
#define UL_SRC_UL_PACK_CONTROLLERWRAPPER_H_

#include "MotionAdapter.h"
#include <ul/controller/Controller.h>

class ControllerWrapper :  public ::ul::controller::Controller {
 public:
  ControllerWrapper(const std::string& config_path);
  ~ControllerWrapper();

  bool goHome_adapter(int arm_type);

  bool goBack_adapter();

  ::ul::math::Vector getJointActualPosition_adapter(int arm_type);

  ::ul::math::Vector getJointActualVelocity_adapter(int arm_type);

  int getInverseKinematics_adapter(const ::Eigen::VectorXd &pose, ::ul::math::Real q7, ::std::vector<::std::vector<Eigen::VectorXd>>& q, ::std::vector<::std::vector<double>>& phi, int arm_type);

  int isSingular_adapter(const Eigen::VectorXd &q, int arm_type);

  bool setJointModeOperation_adapter(const ::ul::std17::RobotRunMode& run_mode, int arm_type);

  bool setControllerMode_adapter(::ul::controller::ControllerMode &mode);

  bool setControllerPara_adapter(::ul::controller::ParasList &para_label, const Eigen::VectorXd &para, int arm_type);

  ::ul::math::Vector getControllerPara_adapter(::ul::controller::ParasList &para_label, int arm_type);

  bool setJointPosition_adapter(const Eigen::VectorXd &q, int arm_type);

  bool setJointTorque_adapter(const Eigen::VectorXd &tau, int arm_type);

  bool teachMode_adapter(int arm_type);

  bool endTeachMode_adapter(int arm_type);

  bool loadCSVData_adapter(const ::std::vector<::std::vector<::ul::math::Real>>&& data, int arm_type);

  bool moveJ_adapter(const Eigen::VectorXd &q, ::ul::math::Real speed, ::ul::math::Real acceleration,  bool asynchronous, int arm_type);

  bool moveJ_adapter(const std::vector<std::vector<::ul::math::Real>>&& path, const ::ul::math::Real& time, bool asynchronous, int arm_type);

  bool moveJ_adapter(const std::vector<std::vector<::ul::math::Real>>&& path, const std::vector<::ul::math::Real>& time, bool asynchronous, int arm_type);

  bool moveJ_adapter(const Eigen::VectorXd &q, const Eigen::VectorXd &qd, ::ul::math::Real freq, bool asynchronous, int arm_type);

  bool moveJ_adapter(const ::Eigen::VectorXd &pose, ::ul::math::Real q7, ::ul::math::Real speed, ::ul::math::Real acceleration, bool asynchronous, int arm_type);

  bool moveL_adapter(const ::Eigen::VectorXd &pose, ::ul::math::Real speed, ::ul::math::Real acceleration, bool asynchronous, int arm_type);

  bool moveL_adapter(const std::vector<std::vector<::ul::math::Real>> &path, const double &time, const bool& asynchronous, int arm_type);

  bool moveL_adapter(const std::vector<std::vector<::ul::math::Real>> &path, const std::vector<::ul::math::Real>& time, const bool& asynchronous, int arm_type);

  bool servoJ_adapter(const Eigen::VectorXd &q, double speed, double acceleration, double time, double lookahead_time, double gain, int arm_type);

  bool servoL_adapter(const ::Eigen::VectorXd &pose, double speed, double acceleration, double time, double lookahead_time, double gain, int arm_type);
  
  bool speedJ_adapter(const Eigen::VectorXd &qd, double acceleration, double time, int arm_type);

  bool speedL_adapter(const ::Eigen::VectorXd &xd, const double& acceleration, const double& time, int arm_type);

  void driver2joint_adapter(::ul::math::Vector& q, ::ul::math::Vector& qd, ::ul::math::Vector& tau);

  void joint2driver_adapter(::ul::math::Vector& p, ::ul::math::Vector& pd, ::ul::math::Vector& tee);

  // 检查是否是Eigen类型
  template<typename T>
  struct is_eigen : std::false_type {};

  template<typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
  struct is_eigen<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>> : std::true_type {};

  template <typename T>
  bool validateJointInput(T param, int type) {
    if(!adapter)
      throw std::runtime_error("Controller not initialized. Please check the robot name in urdf.");

    if(!adapter->validateJointInput(param.size(), type)) {
      std::cout << RED << "[Error] Invalid joint input for arm type "
                << type << "!" << RESET << std::endl;
      return false;
    }
    return true;
  }

  template <typename T>
  bool validateCartesianInput(T param, int type) {
    if(!adapter)
      throw std::runtime_error("Controller not initialized. Please check the robot name in urdf.");

    if constexpr (is_eigen<T>::value) {
      if (1 != type && 2 != type) {
        ::std::cout << RED << "[Error] arm_type is wrong!" << RESET << std::endl;
        return false; 
      }
    }
    if(!adapter->validateCartesianInput(param.size(), type)) {
      std::cout << RED << "[Error] Invalid Cartesian input for arm type "
                << type << "!" << RESET << std::endl;
      return false;
    }
    return true;
  }


 private:
  ::std::unique_ptr<MotionStrategy> adapter;

};

#endif  // UL_SRC_UL_PACK_CONTROLLERWRAPPER_H_
