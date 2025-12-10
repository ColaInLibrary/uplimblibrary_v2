/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-11-12
 * @Version       : 0.0.1
 * @File          : InputHandler.h
 ******************************************************************************
 */

#ifndef UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_INPUTHANDLER_H_
#define UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_INPUTHANDLER_H_

#include <functional>
#include <unordered_map>
#include <string>

// 前置声明任务函数
void executeCommand0();
void executeCommand1();
void executeCommand2();
void executeCommand3();
void executeCommand4();
void executeCommand5();
void executeCommand6();

class InputHandler {
public:
  InputHandler();
  void run();

private:
  std::unordered_map<std::string, std::function<void()>> key_map_;

  void initializeKeyMap();
  void printMenu();
  void handleInput(const std::string& input);
};

#endif // UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_INPUTHANDLER_H_
