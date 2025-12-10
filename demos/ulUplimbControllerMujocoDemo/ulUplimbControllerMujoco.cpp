/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 24-01-22
 * @Version       : 0.0.1
 * @File          : ulUplimbControllerMujoco.cpp
 ******************************************************************************
 */

#include "ThreadManager.h"
#include <iostream>
#include <ul/pack/UplimbController.h>

// 全局变量
std::atomic<bool> keepRunning(true);
// 全局变量声明（与原始代码保持一致）
UPLIMB_INPUT_INFO_STRUCT measured_info;
UPLIMB_OUTPUT_INFO_STRUCT command_info;
UPLIMB_VAR_OBSERVE var_info;

const char* xml_model_path = "/home/ah/Documents/cpp_code/UplimbLibraryPy/examples/models/URDF-H1_Pro/urdf/scene_with_table.xml";

int main() {
  // 初始化Uplimb控制器
  init("/home/ah/Documents/cpp_code/uplimblibrary_v2/config/robot_define_upper_body.yaml");

  ThreadManager threadManager;
  threadManager.startAllThreads();
  threadManager.waitForAllThreads();

  std::cout << ">>>>> Program terminated." << std::endl;
  return 0;
}