/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-11-12
 * @Version       : 0.0.1
 * @File          : RobotAdapter.h
 ******************************************************************************
 */

#ifndef UL_SRC_UL_CONTROLLER_ROBOTADAPTER_H_
#define UL_SRC_UL_CONTROLLER_ROBOTADAPTER_H_

#include "Body.h"
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <initializer_list>

namespace ul {
namespace controller {

// Robot类 - 代表整个机器人
class Robot {
public:
  std::string name;
  std::vector<std::unique_ptr<Body>> bodies;

  // 构造函数
  Robot(const std::string& robotName) : name(robotName) {}

  // 添加身体部分的方法
  void addBody(const std::string& bodyName, int dof) {
    bodies.push_back(std::make_unique<Body>(bodyName, dof));
  }

  // 可以添加其他通用方法
  void printInfo() const {
    std::cout << "Robot: " << name << std::endl;
    std::cout << "Number of bodies: " << bodies.size() << std::endl;
    for (const auto& body : bodies) {
      std::cout << "  - " << body->getName() << ": " << body->getDof() << " DOF" << std::endl;
    }
  }

  virtual ~Robot() = default;  // 虚析构函数，便于扩展
};



// 更易读的配置定义方式
class RobotFactory {
private:
  struct RobotDefinition {
    struct BodyDef {
      const char* name;
      int dof;
    };

    const char* robotName;
    std::initializer_list<BodyDef> bodies;
  };

  static const std::initializer_list<RobotDefinition> robotDefinitions;

public:
  //  静态成员函数
  static std::unique_ptr<Robot> createRobot(const std::string& robotName) {
    for (const auto& def : robotDefinitions) {
      if (robotName == def.robotName) {
        auto robot = std::make_unique<Robot>(robotName);
        for (const auto& bodyDef : def.bodies) {
          robot->addBody(bodyDef.name, bodyDef.dof);
        }
        return robot;
      }
    }
    throw std::invalid_argument("Unknown robot type: " + robotName);
  }
};
}
}
#endif // UL_SRC_UL_CONTROLLER_ROBOTADAPTER_H_
