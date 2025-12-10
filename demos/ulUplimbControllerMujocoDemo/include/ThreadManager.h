/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-11-12
 * @Version       : 0.0.1
 * @File          : ThreadManager.h
 ******************************************************************************
 */

#ifndef UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_THREADMANAGER_H_
#define UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_THREADMANAGER_H_

#include <atomic>
#include <thread>
#include <functional>

extern std::atomic<bool> keepRunning;

class ThreadManager {
public:
  ThreadManager();
  ~ThreadManager();

  void startAllThreads();
  void waitForAllThreads();
  void stopAllThreads();

private:
  std::thread socketThread_;
  std::thread rtThread_;
  std::thread keyPressThread_;
  std::thread plotThread_;

  void socketThreadFunction(std::atomic<bool>* keep_running);
  void rtThreadFunction();
  void keyPressThreadFunction();
  void plotThreadFunction(std::atomic<bool>* keep_running);
};
#endif // UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_THREADMANAGER_H_
