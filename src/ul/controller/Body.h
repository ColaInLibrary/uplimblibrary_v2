/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-11-11
 * @Version       : 0.0.1
 * @File          : Body.h
 ******************************************************************************
 */

#ifndef UL_SRC_UL_CONTROLLER_BODY_H_
#define UL_SRC_UL_CONTROLLER_BODY_H_

#include <string>
#include "JointsMotion.h"
#include "JointsSecurity.h"
#include "CartesianMotion.h"

namespace ul {
namespace controller {
class Body {
  public:
    Body(std::string  bodyName, int dof);
    // 使用 = default 让编译器生成所有特殊成员函数
    Body(const Body&) = default;
    Body(Body&&) = default;
    Body& operator=(const Body&) = default;
    Body& operator=(Body&&) = default;
    ~Body() = default;

    const std::string& getName() const;

    int getDof() const;

  private:
    ::std::string name;
    int dof;
    JointsMotion joints_motion;
    JointsSecurity joints_security;

};
}
}

#endif // UL_SRC_UL_CONTROLLER_BODY_H_
