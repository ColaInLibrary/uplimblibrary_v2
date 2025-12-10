/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 24-01-22
 * @Version       : 0.0.1
 * @File          : ulUplimbControllerMujoco.cpp
 ******************************************************************************
 */

#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <unistd.h>
#include <pwd.h>
#include <ul/pack/UplimbController.h>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
//#include "plot.h"
#include "plot_pipe.h"
#include "task.h"
#include <unordered_map>


::std::atomic<bool> keepRunning(true);

const char *xml_model_path =
    "/home/ah/Documents/cpp_code/UplimbLibraryPy/examples/models/URDF-H1_Pro/urdf/"
    "scene.xml";


// 全局变量
mjModel* m = nullptr;
mjData* d = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;
bool button_left = false, button_middle = false, button_right = false;
double lastx = 0;
double lasty = 0;
UPLIMB_INPUT_INFO_STRUCT measured_info;
UPLIMB_OUTPUT_INFO_STRUCT command_info;
UPLIMB_VAR_OBSERVE var_info;
int dof = 0;
unsigned long long int ticks = 0;

void DataExchange(const mjModel *m, mjData *d, bool isPostionControl) {
  // 获取六维力信息
  std::vector<const char *> force_sensor_names = {"TCP_L_force_sensor",  "TCP_R_force_sensor"};
  std::vector<const char *> torque_sensor_names = {"TCP_R_torque_sensor", "TCP_R_torque_sensor"};
  std::vector<Eigen::VectorXd> ft_wrench(2, Eigen::VectorXd::Zero(6));
  for (int i = 0; i < force_sensor_names.size(); ++i) {
    int force_sensor_id = mj_name2id(m, mjOBJ_SENSOR, force_sensor_names[i]);
    int torque_sensor_id = mj_name2id(m, mjOBJ_SENSOR, torque_sensor_names[i]);
    mjtNum* force = d->sensordata + m->sensor_adr[force_sensor_id];
    mjtNum* torque = d->sensordata + m->sensor_adr[torque_sensor_id];
    for (int j = 0; j < 3; ++j) {
      // 给右手力传感器赋值
      ft_wrench[i][j] = force[j];
      ft_wrench[i][j+3] = torque[j];
    }
  }

//  getFTSensorData(ft_wrench);
//  std::cout << "wrench: " << wrench[0] << " " << wrench[1] << " " << wrench[2] << " " << wrench[3] << " " << wrench[4] << " " << wrench[5] << std::endl;


  // 更新仿真时间
  ticks++;
  measured_info.upTime = ticks;

  // 获取腰部仿真数据
  measured_info.act_pos[dof-1] = d->qpos[0];
  measured_info.act_vel[dof-1] = d->qvel[0];
  measured_info.act_acc[dof-1] = d->qacc[0];

  // 获取颈部仿真数据
  measured_info.act_pos[dof-2] = d->qpos[dof-1];
  measured_info.act_pos[dof-3] = d->qpos[dof-2];
  measured_info.act_vel[dof-2] = d->qvel[dof-1];
  measured_info.act_vel[dof-3] = d->qvel[dof-2];
  measured_info.act_acc[dof-2] = d->qacc[dof-1];
  measured_info.act_acc[dof-3] = d->qacc[dof-2];

  // 获取左臂、右臂仿真数据
  for (int i = 0; i < 7; ++i) {
    measured_info.act_pos[i] = d->qpos[i+7+1];
    measured_info.act_vel[i] = d->qvel[i+7+1];
    measured_info.act_acc[i] = d->qacc[i+7+1];
    measured_info.act_pos[i+7] = d->qpos[i+1];
    measured_info.act_vel[i+7] = d->qvel[i+1];
    measured_info.act_acc[i+7] = d->qacc[i+1];
  }

  if (!isPostionControl) {
    measured_info.act_tau[dof-1] = d->ctrl[0];
    measured_info.act_tau[dof-2] = d->ctrl[dof-1];
    measured_info.act_tau[dof-3] = d->ctrl[dof-2];
    for (int i = 0; i < 7; ++i) {
      measured_info.act_tau[i] = d->ctrl[i+7+1];
      measured_info.act_tau[i+7] = d->ctrl[i+1];
    }
  }
  // 仿真和实物调用相同的接口
  getDriverInfo(&measured_info);
  sendDriverInfo(&command_info);

  // 更新仿真数据
  if (isPostionControl) {
    for (int i = 0; i < 7; ++i) {
      d->qpos[i+1] = command_info.cmd_pos[i+7];
      d->qvel[i+1] = command_info.cmd_vel[i+7];
      d->qpos[i+7+1] = command_info.cmd_pos[i];
      d->qvel[i+7+1] = command_info.cmd_vel[i];
    }
  }

  d->qpos[dof-1] = command_info.cmd_pos[dof-2];
  d->qpos[dof-2] = command_info.cmd_pos[dof-3];
  d->qvel[dof-1] = command_info.cmd_vel[dof-2];
  d->qvel[dof-2] = command_info.cmd_vel[dof-3];
  d->qpos[0] = command_info.cmd_pos[dof-1];
  d->qvel[0] = command_info.cmd_vel[dof-1];

  if (!isPostionControl) {
    ::Eigen::VectorXd Kp(getDof()), Kd(getDof());
    Kp = ::Eigen::VectorXd::Constant(getDof(), 0);
    Kd = ::Eigen::VectorXd::Constant(getDof(), 0);

//    PdControl(Kp, Kd, 0x0F);
    for (int i = 0; i < 7; ++i) {
      d->ctrl[i+1] = command_info.cmd_tau[i+7];
      d->ctrl[i+7+1] = command_info.cmd_tau[i];
    }
    d->ctrl[dof-1] = command_info.cmd_tau[dof-2];
    d->ctrl[dof-2] = command_info.cmd_tau[dof-3];
    d->ctrl[0] = command_info.cmd_pos[dof-1];
  }
}
// 控制器函数 - 添加阻力
void myController(const mjModel* m, mjData* d) {
  DataExchange(m, d, false);
}

void myPositionController(const mjModel* m, mjData* d) {
//  mju_copy(d->qpos, m->key_qpos, m->nq);
  DataExchange(m, d, true);
  mju_zero(d->qvel, m->nv);  // 将速度置零

}

// 键盘回调
void keyboardCB(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // backspace: 重置仿真
  if(act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
}

// 鼠标按钮回调
void mouseButtonCB(GLFWwindow* window, int button, int act, int mods) {
  // 更新按钮状态
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
  // 更新鼠标位置
  glfwGetCursorPos(window, &lastx, &lasty);
}

// 鼠标移动回调
void cursorPosCB(GLFWwindow* window, double xpos, double ypos) {
  // 计算鼠标位移并保存
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // 没有按钮按下: 不做任何操作
  if(!button_left && !button_middle && !button_right)
    return;

  // 获取当前窗口大小
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // 获取shift键状态
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // 根据鼠标按钮确定操作
  mjtMouse action;
  if(button_right)
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  else if(button_left)
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  else
    action = mjMOUSE_ZOOM;

  // 移动相机
  mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}

// 滚轮回调
void scrollCB(GLFWwindow* window, double xoffset, double yoffset) {
  // 模拟垂直鼠标移动 = 窗口高度的5%
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


// 刷新线程函数
void rtThread() {
  // ============================================================================================
  // ============================= Part1: 初始化MuJoCo ==========================================
  // ============================================================================================
  // 加载XML模型文件
  char error[1000] = "";
  m = mj_loadXML(xml_model_path, nullptr, error, 1000);
  if(!m) {
    std::cerr << "Error loading model: " << error << std::endl;
    return;
  }

  // 创建MuJoCo数据结构
  d = mj_makeData(m);

  // 打印 keyframe 信息
  printf("Total keyframes: %d\n", m->nkey);
  for (int i = 0; i < m->nkey; i++) {
    printf("Keyframe %d: time = %.2f\n", i, m->key_time[i]);
  }

  // 应用第一个 keyframe
  if (m->nkey > 0) {
    mju_copy(d->qpos, m->key_qpos, m->nq);
    // mju_copy(data->qvel, model->key_qvel, model->nv);
    // mju_copy(data->act, model->key_act, model->na);
    mj_forward(m, d);
    printf("Applied keyframe 0 to current state.\n");
  }

  // 打印模型信息
  std::cout << "Model loaded: " << xml_model_path << std::endl;
  std::cout << "Number of DoFs: " << m->nq << std::endl;
  std::cout << "Number of keyframes: " << m->nkey << std::endl;

  // 如果有关键帧，使用第一个关键帧作为初始状态
  if(m->nkey > 0) {
    mju_copy(d->qpos, m->key_qpos, m->nq);
    mj_forward(m, d);
  }
  mj_inverse(m, d);
  auto gravity_torque = d->qfrc_bias;
  std::cout << "Gravity torque: " << std::endl;
  for (int i = 0; i < m->nv; ++i) {
    std::cout << gravity_torque[i] << " ";
  }
  std::cout << std::endl;


  // ============================================================================================
  // ============================= Part2: 初始化可视化 ==========================================
  // ============================================================================================
  // 初始化GLFW
  if(!glfwInit()) {
    mju_error("Could not initialize GLFW");
    return;
  }

  // 创建窗口，设置OpenGL上下文，请求垂直同步
  GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo Simulation", nullptr, nullptr);
  if(!window) {
    glfwTerminate();
    mju_error("Could not create GLFW window");
    return;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // 初始化可视化数据结构
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // 创建场景和上下文
  mjv_makeScene(m, &scn, 10000);              // 为10000个对象预留空间
  mjr_makeContext(m, &con, mjFONTSCALE_150);  // 模型特定的上下文

  // 安装GLFW鼠标和键盘回调
  glfwSetKeyCallback(window, keyboardCB);
  glfwSetCursorPosCallback(window, cursorPosCB);
  glfwSetMouseButtonCallback(window, mouseButtonCB);
  glfwSetScrollCallback(window, scrollCB);

  // 设置相机位置 - 固定在距离原点3米处
  cam.type = mjCAMERA_FREE;
  cam.lookat[0] = 0.0;     // 看向原点x
  cam.lookat[1] = 0.0;     // 看向原点y
  cam.lookat[2] = 0.5;     // 看向原点z
  cam.distance = 3.0;      // 3米距离
  cam.azimuth = 180.0;      // 方位角45度
  cam.elevation = -20.0;   // 俯仰角-30度

  // 设置可视化选项
  opt.frame = mjFRAME_WORLD;

  // ============================================================================================
  // ============================= Part3: 主仿真循环 ============================================
  // ============================================================================================
  m->opt.timestep = 0.001;  // 设置仿真时间步长
  glfwSwapInterval(0);
  // 在主循环前设置
//  m->vis.global.offwidth = viewport.width;
//  m->vis.global.offheight = viewport.height;
  while(!glfwWindowShouldClose(window) && keepRunning) {
    // 1. 仿真步进
    double simstart = d->time;
//    while(d->time - simstart < 1.0/60.0) {
//       myController(m, d);  // 应用控制器
////      myPositionController(m, d);
//      mj_step(m, d);
//    }
    myController(m, d);  // 应用控制器
                         //      myPositionController(m, d);
    mj_step(m, d);



    // 2. 渲染
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // 更新场景并渲染
    mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // 交换OpenGL缓冲区（由于垂直同步，这是阻塞调用）
    glfwSwapBuffers(window);

    // 处理挂起的GUI事件，调用GLFW回调
    glfwPollEvents();
  }

  // ============================================================================================
  // ============================= Part4: 清理 ==================================================
  // ============================================================================================
  mjv_freeScene(&scn);
  mjr_freeContext(&con);
  mj_deleteData(d);
  mj_deleteModel(m);
  glfwTerminate();
  keepRunning = false;
  printf("===== RT Task finished!\n");
}

// 按键线程函数
void keyPressThread() {
  ::std::unordered_map<::std::string, ::std::function<void()>> key_map = {
      { "0", executeCommand0},
      { "1", executeCommand1},
      { "2", executeCommand2},
      { "3", executeCommand3},
      { "4", executeCommand4},
      { "5", executeCommand5},
      { "6", executeCommand6},
      { "q", [&]() { keepRunning = false; }},
  };
  ::std::string input;
  while (keepRunning) {
    ::std::cout << "Press Enter to trigger an event" << ::std::endl
                << "0: moveJ_zero\t 1: moveJ_home\t 2: moveJ_path\t 3: speedJ\t 4: speedStop\t 5: CSV\t "
                << ::std::endl
                << "q: quit " << ::std::endl;
    ::std::getline(::std::cin, input);

    auto it = key_map.find(input);
    if (it != key_map.end()) {
      it->second();
    } else  {
      ::std::cout << "Key pressed! Triggered an event." << ::std::endl;
    }
  }
  printf("===== Key Press Task finished!\n");
}

int main() {
//  init();
  init("/home/ah/Documents/cpp_code/UplimbLibrary/config/robot_define_upper_body.yaml");
//  init("/home/ah/Documents/cpp_code/UplimbLibrary/config/robot_define_dual_8dof.yaml");
  dof = getDof();
  // 启动刷新线程
  ::std::thread rt(rtThread);
  // 启动按键线程
  ::std::thread keyPress(keyPressThread);
  // 启动plot线程
  ::std::thread plot_pi(zmq_publisher, &keepRunning);
  // 启动socket线程


  // 等待两个线程结束
  rt.join();
  keyPress.join();
  plot_pi.join();

  ::std::cout << ">>>>> Program terminated."  << ::std::endl;
  return 0;
}