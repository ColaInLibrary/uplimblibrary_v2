/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-11-12
 * @Version       : 0.0.1
 * @File          : Visualization.cpp
 ******************************************************************************
 */

#include "Visualization.h"
#include <iostream>
#include <atomic>

// 全局变量声明
extern std::atomic<bool> keepRunning;
extern mjModel* m;
extern mjData* d;

// 静态成员变量定义
bool Visualization::button_left_ = false;
bool Visualization::button_middle_ = false;
bool Visualization::button_right_ = false;
double Visualization::lastx_ = 0;
double Visualization::lasty_ = 0;

Visualization::Visualization() : m_(nullptr), d_(nullptr) {}

Visualization::~Visualization() {
  shutdown();
}

bool Visualization::initialize(mjModel* m, mjData* d) {
  m_ = m;
  d_ = d;

  if (!initializeGLFW()) {
    return false;
  }

  if (!createWindow()) {
    return false;
  }

  initializeVisualizationObjects();
  setupCallbacks();
  setupCamera();

  return true;
}

bool Visualization::initializeGLFW() {
  if (!glfwInit()) {
    std::cerr << "Could not initialize GLFW" << std::endl;
    return false;
  }
  return true;
}

bool Visualization::createWindow() {
  window_ = glfwCreateWindow(1200, 900, "MuJoCo Simulation", nullptr, nullptr);
  if (!window_) {
    std::cerr << "Could not create GLFW window" << std::endl;
    glfwTerminate();
    return false;
  }

  glfwMakeContextCurrent(window_);
  glfwSwapInterval(0); // 设置为0以获得最大渲染性能

  // 设置用户指针以便在静态回调中访问实例
  glfwSetWindowUserPointer(window_, this);

  return true;
}

void Visualization::initializeVisualizationObjects() {
  if (!m_) {
    std::cerr << "Error: mjModel is null when initializing visualization" << std::endl;
    return;
  }

  // 初始化默认对象
  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  mjv_defaultScene(&scn_);
  mjr_defaultContext(&con_);

  // 创建场景和上下文
  mjv_makeScene(m_, &scn_, 10000);              // 为10000个对象预留空间
  mjr_makeContext(m_, &con_, mjFONTSCALE_150);  // 模型特定的上下文
}

void Visualization::setupCallbacks() {
  glfwSetKeyCallback(window_, Visualization::keyboardCallback);
  glfwSetCursorPosCallback(window_, Visualization::cursorPosCallback);
  glfwSetMouseButtonCallback(window_, Visualization::mouseButtonCallback);
  glfwSetScrollCallback(window_, Visualization::scrollCallback);
}

void Visualization::setupCamera() {
  cam_.type = mjCAMERA_FREE;
  cam_.lookat[0] = 0.0;     // 看向原点x
  cam_.lookat[1] = 0.0;     // 看向原点y
  cam_.lookat[2] = 0.5;     // 看向原点z
  cam_.distance = 3.0;      // 3米距离
  cam_.azimuth = 180.0;     // 方位角180度
  cam_.elevation = -20.0;   // 俯仰角-20度

  // 设置可视化选项
  opt_.frame = mjFRAME_WORLD;
}

void Visualization::render() {
  if (!window_ || !m_ || !d_) {
    return;
  }

  // 获取帧缓冲区大小
  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

  // 更新场景并渲染
  mjv_updateScene(m_, d_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
  mjr_render(viewport, &scn_, &con_);

  // 交换OpenGL缓冲区
  glfwSwapBuffers(window_);
}

bool Visualization::shouldClose() {
  return !window_ || glfwWindowShouldClose(window_) || !keepRunning;
}

void Visualization::pollEvents() {
  if (window_) {
    glfwPollEvents();
  }
}

void Visualization::shutdown() {
  if (window_) {
    // 在销毁窗口前清除回调，避免访问已释放的内存
    glfwSetKeyCallback(window_, nullptr);
    glfwSetCursorPosCallback(window_, nullptr);
    glfwSetMouseButtonCallback(window_, nullptr);
    glfwSetScrollCallback(window_, nullptr);

    glfwDestroyWindow(window_);
    window_ = nullptr;
  }

  if (scn_.ngeom > 0) {
    mjv_freeScene(&scn_);
  }

  if (con_.currentBuffer) {
    mjr_freeContext(&con_);
  }

  glfwTerminate();
}

void Visualization::moveCamera(mjtMouse action, double dx, double dy) {
  if (!m_ || !d_) {
    return;
  }
  mjv_moveCamera(m_, action, dx, dy, &scn_, &cam_);
}

// 静态回调函数实现
void Visualization::keyboardCallback(GLFWwindow* window, int key, int scancode, int act, int mods) {
  Visualization* vis = static_cast<Visualization*>(glfwGetWindowUserPointer(window));
  if (!vis || !vis->m_ || !vis->d_) {
    return;
  }

  // backspace: 重置仿真
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
    mj_resetData(vis->m_, vis->d_);
    mj_forward(vis->m_, vis->d_);
    std::cout << "Simulation reset" << std::endl;
  }

  // ESC键退出
  if (act == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
    keepRunning = false;
  }

  // 空格键暂停/继续
  if (act == GLFW_PRESS && key == GLFW_KEY_SPACE) {
    static bool paused = false;
    paused = !paused;
    std::cout << (paused ? "Simulation paused" : "Simulation resumed") << std::endl;
  }
}

void Visualization::mouseButtonCallback(GLFWwindow* window, int button, int act, int mods) {
  Visualization* vis = static_cast<Visualization*>(glfwGetWindowUserPointer(window));
  if (!vis) {
    return;
  }

  // 更新按钮状态
  vis->button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  vis->button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  vis->button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // 更新鼠标位置
  glfwGetCursorPos(window, &vis->lastx_, &vis->lasty_);
}

void Visualization::cursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
  Visualization* vis = static_cast<Visualization*>(glfwGetWindowUserPointer(window));
  if (!vis || !vis->m_ || !vis->d_) {
    return;
  }

  // 计算鼠标位移
  double dx = xpos - vis->lastx_;
  double dy = ypos - vis->lasty_;
  vis->lastx_ = xpos;
  vis->lasty_ = ypos;

  // 没有按钮按下: 不做任何操作
  if (!vis->button_left_ && !vis->button_middle_ && !vis->button_right_) {
    return;
  }

  // 获取当前窗口大小
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // 获取shift键状态
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // 根据鼠标按钮确定操作
  mjtMouse action;
  if (vis->button_right_) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (vis->button_left_) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // 移动相机
  vis->moveCamera(action, dx / height, dy / height);
}

void Visualization::scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
  Visualization* vis = static_cast<Visualization*>(glfwGetWindowUserPointer(window));
  if (!vis || !vis->m_ || !vis->d_) {
    return;
  }

  // 模拟垂直鼠标移动 = 窗口高度的5%
  vis->moveCamera(mjMOUSE_ZOOM, 0, -0.05 * yoffset);
}