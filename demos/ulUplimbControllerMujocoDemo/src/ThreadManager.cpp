/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-11-12
 * @Version       : 0.0.1
 * @File          : ThreadManager.cpp
 ******************************************************************************
 */

#include "ThreadManager.h"
#include "MujocoSimulator.h"
#include "Visualization.h"
#include "InputHandler.h"
#include "plot_pipe.h"
#include <iostream>
#include <ul/pack/UplimbController.h>

extern const char* xml_model_path;

ThreadManager::ThreadManager() = default;

ThreadManager::~ThreadManager() {
  stopAllThreads();
}

void ThreadManager::startAllThreads() {
  keepRunning = true;
  socketThread_ = std::thread(&ThreadManager::socketThreadFunction, this, &keepRunning);
  rtThread_ = std::thread(&ThreadManager::rtThreadFunction, this);
  keyPressThread_ = std::thread(&ThreadManager::keyPressThreadFunction, this);
  plotThread_ = std::thread(&ThreadManager::plotThreadFunction, this, &keepRunning);
}

void ThreadManager::waitForAllThreads() {
  if (socketThread_.joinable()) socketThread_.join();
  if (rtThread_.joinable()) rtThread_.join();
  if (keyPressThread_.joinable()) keyPressThread_.join();
  if (plotThread_.joinable()) plotThread_.join();
}

void ThreadManager::stopAllThreads() {
  keepRunning = false;
  waitForAllThreads();
}

void ThreadManager::socketThreadFunction(std::atomic<bool>* keep_running) {
  std::cout << "===== Thread1: Socket Task started!" << std::endl;
  serverRun(keep_running);
  std::cout << "===== Thread1: Socket Task finished!" << std::endl;
}

void ThreadManager::rtThreadFunction() {
  std::cout << "===== Thread2: RT Task started!" << std::endl;
  MujocoSimulator simulator;
  Visualization visualization;

  if (!simulator.initialize(xml_model_path)) {
    std::cerr << "Failed to initialize MuJoCo simulator" << std::endl;
    return;
  }

  if (!visualization.initialize(simulator.getModel(), simulator.getData())) {
    std::cerr << "Failed to initialize visualization" << std::endl;
    return;
  }

  std::cout << "Robot Dof from Controller: " << getDof() << std::endl;
  // 主仿真循环
  while (!visualization.shouldClose() && keepRunning) {
    simulator.step();
    visualization.render();
    visualization.pollEvents();
  }

  visualization.shutdown();
  simulator.shutdown();
  std::cout << "===== Thread2: RT Task finished!" << std::endl;
}

void ThreadManager::keyPressThreadFunction() {
  std::cout << "===== Thread3: Key Press Task started!" << std::endl;
  InputHandler inputHandler;
  inputHandler.run();
  std::cout << "===== Thread3: Key Press Task finished!" << std::endl;
}

void ThreadManager::plotThreadFunction(std::atomic<bool>* keep_running) {
  std::cout << "===== Thread4: Plot Task started!" << std::endl;
  zmq_publisher(keep_running);
  std::cout << "===== Thread4: Plot Task finished!" << std::endl;
}
