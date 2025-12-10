#include <stdint.h>

namespace ul {
namespace std17 {
enum class ErrorType : uint16_t {
    // 正确
    SUCCESS                 = 0x0000,

    // 规划错误 
    INPUT_PARA_ERROR        = 0x1100,  // 输入参数错误
    PLAN_ERROR              = 0x1200,  // 轨迹规划错误

    // 数据错误 
    POS_JUMP_ERROR          = 0x2100,  // 位置跳变错误
    VEL_JUMP_ERROR          = 0x2200,  // 速度跳变错误
    ACC_JUMP_ERROR          = 0x2300,  // 加速度跳变错误
    POS_NAN_ERROR           = 0x2400,  // 位置非数错误
    VEL_NAN_ERROR           = 0x2500,  // 速度非数错误
    ACC_NAN_ERROR           = 0x2600,  // 加速度非数错误
    TAU_NAN_ERROR           = 0x2700,  // 力矩非数错误

    // 限幅错误 
    JOINT_POS_LIMIT_ERROR   = 0x3100,  // 关节位置限幅保护错误
    JOINT_VEL_LIMIT_ERROR   = 0x3200,  // 关节速度限幅保护错误
    JOINT_TAU_LIMIT_ERROR   = 0x3300,  // 关节力矩限幅保护错误

    // 运动学错误 
    ROTATION_MATRIX_ERROR   = 0x4100,  // 运动学错误：旋转矩阵错误
    IK_NO_SOLUTION_ERROR    = 0x4200,  // 运动学错误：逆运动学无解
    JACOBIAN_SINGULAR_ERROR = 0x4300,  // 运动学错误：雅可比矩阵奇异

    // 通信错误
    COMM_ERROR              = 0x5100,  // 通信错误

    // 其他错误
    DENOMINATOR_ERROR       = 0x6100,  // 分母为零错误
    SENSOR_ERROR            = 0x6200,  // 传感器错误
};


enum class JointID : uint16_t {
    JOINT_1  = 1,
    JOINT_2  = 2,
    JOINT_3  = 3,
    JOINT_4  = 4,
    JOINT_5  = 5,
    JOINT_6  = 6,
    JOINT_7  = 7,
    JOINT_8  = 8,
    JOINT_9  = 9,
    JOINT_10 = 10,
    JOINT_11 = 11,
    JOINT_12 = 12,
    JOINT_13 = 13,
    JOINT_14 = 14,
    JOINT_15 = 15,
    JOINT_16 = 16,
    JOINT_17 = 17,
    JOINT_18 = 18,
    JOINT_19 = 19,
    JOINT_20 = 20,
    JOINT_21 = 21,
    JOINT_22 = 22,
    JOINT_23 = 23,
    JOINT_24 = 24,
    JOINT_25 = 25,
    JOINT_26 = 26,
    JOINT_27 = 27,
    JOINT_28 = 28,
    JOINT_29 = 29,
    JOINT_30 = 30,
    JOINT_31 = 31,
    JOINT_32 = 32,
    JOINT_33 = 33,
    JOINT_34 = 34,
    JOINT_35 = 35,
};

// 返回一个unit16_t类型的错误码
class ErrorCode {
public:
    static uint16_t makeError(ErrorType type, JointID joint) {
        return static_cast<uint16_t>(type) | static_cast<uint16_t>(joint);
    }
    
    static ErrorType getErrorType(uint16_t errorCode) {
        return static_cast<ErrorType>(errorCode & 0xFF00);
    }
    
    static JointID getJointID(uint16_t errorCode) {
        return static_cast<JointID>(errorCode & 0x00FF);
    }
    
    static bool isJointError(uint16_t errorCode, JointID joint) {
        return getJointID(errorCode) == joint;
    }
};
}  // namespace std17
}  // namespace ul