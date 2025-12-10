/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-1-14
 * @Version       : 0.0.1
 * @File          : common.h
 ******************************************************************************
 */

#ifndef UL_SRC_UL_STD_COMMON_H_
#define UL_SRC_UL_STD_COMMON_H_

#include <stdint.h>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/SVD"
#include "eigen3/Eigen/Sparse"

#define RESET "\033[0m"
#define RED "\033[31m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"
#define BLUE "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN "\033[36m"

#define UPLIMB_MAX_DIMENSION 30  // 该数值与嵌入式软件组定义的最大关节数一致

namespace ul {
namespace std17 {
enum class RobotRunMode { PT = 4, CSP = 8, CSV, CST };

enum class RobotCommand { NO_CMD = 0, MOVEJ = 1, MOVEJ_PATH, MOVEL_NULLSPACE, MOVEL, MOVEL_PATH, SPEEDJ, SPEEDL, SPEED_STOP, MOVECSVFILE, MOVEFOURIER, MOVEJ_SPLINE, TEACH, SERVOJ, SERVOL, PROTECTED };

struct JointState {
  double q;
  double qd;
  double qdd;
  double qddd;
  double t;
};

struct MotionParas {
  double jerk;
  double alima;
  double alimd;
  double vlim;
  double Tj1;
  double Tj2;
  double Ta;  // 加速段时间
  double Td;  // 减速段时间
  double Tv;  // 匀速段时间
  double T;   // 总时间
  bool joy;
};

struct CartesianConfig {
  ::std::vector<std::string> arm_type;
  ::std::vector<int> joint_num;
  ::std::vector<std::string> joint_name_first;
  ::std::vector<::std::size_t> joint_idx_first;
  ::std::vector<std::string> ee_name;
  ::std::vector<::std::size_t> ee_idx;
  ::std::vector<::std::vector<::std::size_t>> cartesian_select;  // 关节数量少于6是选择控制哪些自由度
};

}  // namespace std17
}  // namespace ul

#endif  // UL_SRC_UL_STD_COMMON_H_
