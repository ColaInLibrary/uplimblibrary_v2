/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-11-12
 * @Version       : 0.0.1
 * @File          : Visualization.h
 ******************************************************************************
 */

#ifndef UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_VISUALIZATION_H_
#define UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_VISUALIZATION_H_

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

class Visualization {
public:
  Visualization();
  ~Visualization();

  bool initialize(mjModel* m, mjData* d);
  void render();
  bool shouldClose();
  void pollEvents();
  void shutdown();

  // 回调函数
  static void keyboardCallback(GLFWwindow* window, int key, int scancode, int act, int mods);
  static void mouseButtonCallback(GLFWwindow* window, int button, int act, int mods);
  static void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);
  static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

private:
  GLFWwindow* window_ = nullptr;
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  mjvCamera cam_;
  mjvOption opt_;
  mjvScene scn_;
  mjrContext con_;

  // 鼠标状态
  static bool button_left_;
  static bool button_middle_;
  static bool button_right_;
  static double lastx_;
  static double lasty_;

  bool initializeGLFW();
  bool createWindow();
  void initializeVisualizationObjects();
  void setupCallbacks();
  void setupCamera();
  void moveCamera(mjtMouse action, double dx, double dy);
};
#endif // UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_VISUALIZATION_H_
