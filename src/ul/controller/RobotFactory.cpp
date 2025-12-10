/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-11-12
 * @Version       : 0.0.1
 * @File          : RobotFactory.cpp
 ******************************************************************************
 */
#include "RobotFactory.h"


namespace ul::controller {
// 配置定义
const std::initializer_list<RobotFactory::RobotDefinition>
    RobotFactory::robotDefinitions = {
        {
            "A2",  // 机器人名称
            {
                {"left_arm", 7},  // 身体名称, DOF
                {"right_arm", 7}
            }
        },
        {
            "A3",
            {
                {"left_arm", 8},
                {"right_arm", 8}
            }
        },
        {
            "U1",
            {
                {"left_arm", 7},
                {"right_arm", 7},
                {"head", 2},
                {"waist", 1}
            }
        },
        {
            "WA1",
            {
                {"left_arm", 7},
                {"right_arm", 7},
                {"head", 2},
                {"waist", 3}
            }
        },
        {
            "WA2",
            {
                {"left_arm", 8},
                {"right_arm", 8},
                {"head", 2},
                {"waist", 4}
            }
        },
        {
            "180Robot",
            {
                {"left_arm", 7},
                {"right_arm", 3},
                {"head", 2},
                {"waist", 3}
            }
        }
};
}

