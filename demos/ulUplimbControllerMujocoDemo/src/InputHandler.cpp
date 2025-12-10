/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-11-12
 * @Version       : 0.0.1
 * @File          : InputHandler.cpp
 ******************************************************************************
 */

#include "InputHandler.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

// 全局变量声明
extern std::atomic<bool> keepRunning;

// 任务函数声明
void executeCommand0();
void executeCommand1();
void executeCommand2();
void executeCommand3();
void executeCommand4();
void executeCommand5();
void executeCommand6();

InputHandler::InputHandler() {
  initializeKeyMap();
}

void InputHandler::initializeKeyMap() {
  key_map_ = {
      {"0", executeCommand0},
      {"1", executeCommand1},
      {"2", executeCommand2},
      {"3", executeCommand3},
      {"4", executeCommand4},
      {"5", executeCommand5},
      {"6", executeCommand6},
      {"q", [&]() {
         std::cout << "Quit command received. Shutting down..." << std::endl;
         keepRunning = false;
       }},
      {"quit", [&]() {
         std::cout << "Quit command received. Shutting down..." << std::endl;
         keepRunning = false;
       }},
      {"exit", [&]() {
         std::cout << "Exit command received. Shutting down..." << std::endl;
         keepRunning = false;
       }},
      {"help", [&]() {
         printMenu();
       }},
      {"menu", [&]() {
         printMenu();
       }},
      {"", [&]() {
         // 空输入，不做任何操作
         std::cout << "Empty input ignored." << std::endl;
       }}
  };
}

void InputHandler::printMenu() {
  std::cout << "\n==========================================" << std::endl;
  std::cout << "        Uplimb Controller Menu" << std::endl;
  std::cout << "==========================================" << std::endl;
  std::cout << "0: moveJ_zero    - 移动到零位" << std::endl;
  std::cout << "1: moveJ_home    - 移动到Home位" << std::endl;
  std::cout << "2: moveJ_path    - 路径运动" << std::endl;
  std::cout << "3: speedJ        - 速度控制" << std::endl;
  std::cout << "4: speedStop     - 停止运动" << std::endl;
  std::cout << "5: CSV           - CSV文件运动" << std::endl;
  std::cout << "6: (预留功能)    - 预留命令" << std::endl;
  std::cout << "q/quit/exit:     - 退出程序" << std::endl;
  std::cout << "help/menu:       - 显示此菜单" << std::endl;
  std::cout << "==========================================" << std::endl;
  std::cout << "请输入命令编号或名称: ";
  std::cout.flush();
}

void InputHandler::handleInput(const std::string& input) {
  // 转换为小写以便不区分大小写
  std::string lowerInput = input;
  for (char& c : lowerInput) {
    c = std::tolower(c);
  }

  auto it = key_map_.find(lowerInput);
  if (it != key_map_.end()) {
    try {
      std::cout << "执行命令: " << input << std::endl;
      it->second();
      std::cout << "命令执行完成" << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "命令执行出错: " << e.what() << std::endl;
    } catch (...) {
      std::cerr << "命令执行出现未知错误" << std::endl;
    }
  } else {
    std::cout << "未知命令: " << input << std::endl;
    std::cout << "输入 'help' 查看可用命令" << std::endl;
  }
}

void InputHandler::run() {
  std::string input;

  // 显示欢迎信息
  std::cout << "\n=== Uplimb Controller 输入处理线程启动 ===" << std::endl;
  printMenu();

  while (keepRunning) {
    // 检查是否还需要继续运行
    if (!keepRunning) {
      break;
    }

    // 获取用户输入
    std::cout << "\n>> 请输入命令: ";
    if (!std::getline(std::cin, input)) {
      // 输入流错误或EOF
      if (std::cin.eof()) {
        std::cout << "检测到EOF，退出程序..." << std::endl;
        keepRunning = false;
        break;
      }
      std::cin.clear(); // 清除错误状态
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cout << "输入错误，请重新输入" << std::endl;
      continue;
    }

    // 处理输入
    handleInput(input);

    // 小延迟避免过度占用CPU
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::cout << "===== Key Press Task exiting... ===" << std::endl;
}

// 默认的任务函数实现（占位符）
void executeCommand0() {
  std::cout << "执行 moveJ_zero 命令..." << std::endl;
  // 这里应该调用实际的运动控制函数
  // moveJ_zero();
}

void executeCommand1() {
  std::cout << "执行 moveJ_home 命令..." << std::endl;
  // 这里应该调用实际的运动控制函数
  // moveJ_home();
}

void executeCommand2() {
  std::cout << "执行 moveJ_path 命令..." << std::endl;
  // 这里应该调用实际的运动控制函数
  // moveJ_path();
}

void executeCommand3() {
  std::cout << "执行 speedJ 命令..." << std::endl;
  // 这里应该调用实际的运动控制函数
  // speedJ();
}

void executeCommand4() {
  std::cout << "执行 speedStop 命令..." << std::endl;
  // 这里应该调用实际的运动控制函数
  // speedStop();
}

void executeCommand5() {
  std::cout << "执行 CSV 文件运动命令..." << std::endl;
  // 这里应该调用实际的运动控制函数
  // executeCSVMotion();
}

void executeCommand6() {
  std::cout << "执行预留命令 6..." << std::endl;
  // 预留功能实现
}
