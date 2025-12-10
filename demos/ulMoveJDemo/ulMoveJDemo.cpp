/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 24-12-30
 * @Version       : 0.0.1
 * @File          : ulMoveJDemo.cpp
 ******************************************************************************
 */

#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <ul/math/Transform.h>
#include <ul/math/Vector.h>
#include <ul/math/SShapeVelocity.h>
#include <ul/controller/Controller.h>


::std::atomic<bool> keepRunning(true);

// 刷新线程函数
void rtThread(::ul::controller::Controller& controller) {
  while (keepRunning) {
    // 模拟刷新任务
    ::std::cout << "Refreshing..." << ::std::endl;
    // 每 0.001s 刷新一次
    ::std::this_thread::sleep_for(::std::chrono::milliseconds(2000));
    controller.run();
  }
}

// 按键线程函数
void keyPressThread(::ul::controller::Controller& controller) {
  ::std::string input;
  while (keepRunning) {
    ::std::cout << "Press Enter to trigger an event or type 'exit' to quit: " << ::std::endl;
    ::std::getline(::std::cin, input);
    if (input == "esc") {
      keepRunning = false;
    } else {
      ::ul::math::Vector init_q(controller.robot_.getDof());
      init_q.setZero();
      controller.setJointPosition(init_q);
      ::ul::math::Vector target_q(controller.robot_.getDof());
      target_q.setZero();
      controller.moveJ(target_q, 0.5, 1.4, false);
      ::std::cout << "Key pressed! Triggered an event." << ::std::endl;
    }
  }
}

int main() {
  ::ul::controller::Controller controller;

  // 启动刷新线程
  ::std::thread rt(rtThread, ::std::ref(controller));
  // 启动按键线程
  ::std::thread keyPress(keyPressThread, ::std::ref(controller));

  // 等待两个线程结束
  rt.join();
  keyPress.join();

  ::std::cout << "Program terminated." << ::std::endl;
  return 0;
}