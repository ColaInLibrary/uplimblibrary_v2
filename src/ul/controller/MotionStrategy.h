/**
 ******************************************************************************
 * @Description   : 机器人类型适配策略基类
 * @author        : AN Hao
 * @Date          : 25-5-21
 * @Version       : 0.0.1
 * @File          : MotionStrategy.h
 ******************************************************************************
 */

#ifndef UL_SRC_UL_PACK_MOTIONSTRATEGY_H_
#define UL_SRC_UL_PACK_MOTIONSTRATEGY_H_
#include <iostream>
#include <Eigen/Dense>
#include <type_traits>
#include <memory>
#include <string>
#include <cmath>
#include <numeric>
#include <ul/math/Real.h>
#include <ul/math/Vector.h>
#include <ul/std/common.h>
#include "CartesianMotion.h"


#define ARM_JOINT_NUM 7
#define I3_ARM_JOINT_NUM 4
#define A3_ARM_JOINT_NUM 8
#define A2_ARM_JOINT_NUM 8
#define WA3_WAIST_JOINT_NUM 4
#define WA2_WAIST_JOINT_NUM 4
#define R180_ARM_JOINT_NUM 3
#define HEAD_JOINT_NUM 2
#define WAIST_JOINT_NUM 1
#define WHEEL_WAIST_JOINT_NUM 2
#define WHEEL_WAIST_LIFT_JOINT_NUM 1
#define R180_WAIST_JOINT_NUM 2
#define I3_WAIST_JOINT_NUM 3

class Controller; // 前向声明

class MotionStrategy {
 public:
  MotionStrategy() = default;
  virtual ~MotionStrategy() = default;

  ::std::vector<::ul::math::Vector> q_home;

  ::std::vector<int> joint_num;

  ::ul::std17::CartesianConfig cartesian_config; 

  size_t getDof() const { return std::accumulate(joint_num.begin(), joint_num.end(), 0); }

  void jointPathPadding(::std::vector<::std::vector<::ul::math::Real>>& target,
                        const ::std::vector<::std::vector<::ul::math::Real>>& path,
                        int before_index,
                        int after_index,
                        int path_start_index) const {
      const size_t row_count = path.size();

      if (target.size() != row_count) {
          throw std::invalid_argument("[jointPathPadding] Target and path size mismatch");
      }

      const int copy_count = after_index - before_index;
      if (copy_count <= 0) return;

      for (size_t i = 0; i < row_count; ++i) {
          const auto& src_row = path[i];
          auto& tgt_row = target[i];

          if (static_cast<size_t>(path_start_index + copy_count) > src_row.size() || static_cast<size_t>(after_index) > tgt_row.size()) {
              throw std::out_of_range("[jointPathPadding] Copy indices out of bounds");
          }

          std::copy(src_row.begin() + path_start_index, src_row.begin() + path_start_index + copy_count, tgt_row.begin() + before_index);
      }
  }


  void cartesianPathPadding(std::vector<std::vector<::ul::math::Real>>& target,
                        const std::vector<std::vector<::ul::math::Real>>& path,
                        bool isLeft,
                        ::ul::math::Vector6& cmd_pose_last) const {
    target.clear();
    std::vector<::ul::math::Real> point_temp(12);
    for(auto point: path) {
      if (isLeft) {
        std::copy(point.begin(), point.end(), point_temp.begin()); //左臂姿态
        Eigen::Map<Eigen::VectorXd>(point_temp.data() + 6, 6) = cmd_pose_last; //右臂姿态
      } else {
        std::copy(point.begin(), point.end(), point_temp.begin()+6);  //右臂姿态
        Eigen::Map<Eigen::VectorXd>(point_temp.data(), 6) = cmd_pose_last; //左臂姿态
      }
      target.push_back(point_temp);
    }
  }

  bool validateJointInput(const size_t& size, int arm_type) const {
    if (arm_type > (1 << joint_num.size()) - 1 || arm_type < 1) {
      ::std::cout << "[ERROR] arm_type is wrong!" << ::std::endl;
      return false;
    }
    int Size = 0;
    for (int i = 0; i < joint_num.size(); ++i) {
      if(arm_type & (1 << i)) {
        Size += joint_num[i];
      } 
    }
    return Size == size;
  }

  template<typename TargetType, typename SourceType>
  void applyJointInput(TargetType& target, const SourceType& q, int arm_type) const {
      static_assert(
          std::is_same_v<typename TargetType::Scalar, typename SourceType::Scalar>,
          "[ERROR] Source and target must have same scalar type"
      );
      
      if (arm_type > (1 << joint_num.size()) - 1 || arm_type < 1) {
          ::std::cout << "[ERROR] arm_type is wrong!" << ::std::endl;
          return;
      }
      
      int idx = 0, index = 0;
      for (int i = 0; i < joint_num.size(); ++i) {
          if(arm_type & (1 << i)) {
              target.segment(idx, joint_num[i]) = q.segment(index, joint_num[i]);
              index += joint_num[i];
          }
          idx += joint_num[i];
      }
  }

  void applyJointPathInput(std::vector<std::vector<ul::math::Real>>& target,
                            const std::vector<std::vector<ul::math::Real>>& partial_path,
                            const ul::math::Vector& cmd_q_last,
                            int arm_type) const {
    if (arm_type > (1 << joint_num.size()) - 1 || arm_type < 1) {
      std::cout << "[ERROR] arm_type is wrong!" << std::endl;
      return;
    }

    const std::size_t path_size = partial_path.size();
    const std::size_t dof = static_cast<std::size_t>(cmd_q_last.size());

    target.assign(path_size, std::vector<ul::math::Real>(cmd_q_last.data(), cmd_q_last.data() + dof));

    int idx = 0, index = 0;
    for (int i = 0; i < static_cast<int>(joint_num.size()); ++i) {
      if (arm_type & (1 << i)) {
        this->jointPathPadding(target, partial_path, idx, idx + joint_num[i], index);
        index += joint_num[i];
      }
      idx += joint_num[i];
    }
  }

  void applyHomeInput(::ul::math::Vector& target, int arm_type) const {
    if (arm_type > (1 << joint_num.size()) - 1 || arm_type < 1) {
      ::std::cout << "[ERROR] arm_type is wrong!" << ::std::endl;
      return;
    }  
    int index = 0;
    for (int i = 0; i < joint_num.size(); ++i) {
      if(arm_type & (1 << i)) {
        target.conservativeResize(index + joint_num[i]);
        target.segment(index, joint_num[i]) = q_home[i];
        index += joint_num[i];
      } 
    }
  }

  ::ul::math::Vector getJointActualInfo(const ::ul::math::Vector& q, int arm_type) const {
    ::ul::math::Vector target;
    if (arm_type > (1 << joint_num.size()) - 1 || arm_type < 1) {
      ::std::cout << "[ERROR] arm_type is wrong!" << ::std::endl;
      return q;
    }  
    int idx = 0, index = 0;
    for (int i = 0; i < joint_num.size(); ++i) {
      if(arm_type & (1 << i)) {
        target.conservativeResize(index + joint_num[i]);
        target.segment(index, joint_num[i]) = q.segment(idx, joint_num[i]);
        index += joint_num[i];
      }
      idx += joint_num[i];
    }
    return target;
  }

  bool applyJointRunMode(const ::ul::std17::RobotRunMode& run_mode, int arm_type, ::std::vector<uint8_t> &mode_operation) const {
    if (arm_type > (1 << joint_num.size()) - 1 || arm_type < 1) {
      ::std::cout << "[ERROR] arm_type is wrong!" << ::std::endl;
      return false;
    }
    int Size = 0;
    for (int i = 0; i < joint_num.size(); ++i) {
      if(arm_type & (1 << i)) {
        Size += joint_num[i];
      } 
    }

    mode_operation = std::vector<uint8_t>(Size, static_cast<uint8_t>(run_mode));
    return true;
  }
  // 验证笛卡尔输入是否有效
  bool validateCartesianInput(const size_t& size, int arm_type) const {
    switch (arm_type) {
      case 3:  // 所有关节
        return size == 12;
      case 1: case 2: return size == 6;
      default: return false;
    }
  }

  void applyCartesianInput(::std::vector<::ul::math::Vector6>& pose_target, const ::ul::math::Vector6& pose, int arm_type) const {
    switch (arm_type) {
      case 1:  pose_target[0] = pose; break;
      case 2:  pose_target[1] = pose; break;
      default:
        ::std::cout << "[ERROR] arm_type is wrong!" << ::std::endl;
        break;
    }
  }     

  void applyCartesianPathInput(std::vector<std::vector<::ul::math::Real>>& target,
                                   const std::vector<std::vector<::ul::math::Real>>& partial_path,
                                   ::std::vector<::ul::math::Vector6>& cmd_pose_last,
                                   int arm_type) const {
    switch (arm_type) {
      case 3: target = partial_path; break;
      case 1:  this->cartesianPathPadding(target, partial_path, true, cmd_pose_last[1]); break;
      case 2:  this->cartesianPathPadding(target, partial_path, false, cmd_pose_last[0]); break;
      default:
        ::std::cout << "[ERROR] arm_type is wrong!" << ::std::endl;
        break;
    }
  } 

  void applyCartesianConfig() {
    this->cartesian_config.arm_type = {"ARM_LEFT", "ARM_RIGHT"};
    this->cartesian_config.joint_num = {joint_num[0], joint_num[1]};
    this->cartesian_config.ee_idx.resize(2);
    this->cartesian_config.joint_idx_first.resize(2);

    return;
  }
  
  // 用于处理算法顺序和驱动顺序映射
  virtual void driver2alg(::ul::math::Vector& vec) { return; }

  // 用于处理算法顺序和驱动顺序映射
  virtual void alg2driver(::ul::math::Vector& vec) { return; }
  
  // 获取部位名称用于错误信息
//  virtual const char* partName() const = 0;

  // 获取期望的关节数量
//  virtual int expectedSize() const = 0;
};
#endif  // UL_SRC_UL_PACK_MOTIONSTRATEGY_H_
