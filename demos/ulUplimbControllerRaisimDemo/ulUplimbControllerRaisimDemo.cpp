/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 24-01-22
 * @Version       : 0.0.1
 * @File          : ulUplimbControllerDemo.cpp
 ******************************************************************************
 */

#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <unistd.h>
#include <pwd.h>
//#include "UplimbController.h"
#include <ul/pack/UplimbController.h>

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "plot.h"
#include <unordered_map>


::std::atomic<bool> keepRunning(true);

void executeCommand0() {
  ::Eigen::VectorXd target_q(getDof());
  target_q.setZero();
  ::std::cout << getJointActualPositions() << ::std::endl;
  if (!moveJ(target_q, 3, 1.4, false)) return;
}

void executeCommand1() {
  ::Eigen::VectorXd target_q(getDof());
//  target_q << -0.049, -0.508, -0.082, -1.302,  0.809, -0.01, -0.15,
  target_q << -0.223363, -0.38122, 0.189163, -0.819702, 0.566464, -0.0839337, 0.0170038,
      -0.035,  0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
      0, 0;
//  target_q.setZero();
//  target_q[0] = -0.049;
  moveJ(target_q, 3, 1.4, false);
}

void executeCommand2() {
  ::std::vector<::std::vector<double>> path(3, std::vector<double>(getDof(), 0));
  ::Eigen::VectorXd target_q(getDof());
  target_q << -0.223363, -0.38122, 0.189163, -0.819702, 0.566464, -0.0839337, 0.0170038,
      -0.035,  0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
      0, 0;
  for (int i = 0; i < getDof(); ++i) {
    path[0][i] = target_q[i];
  }
  target_q << -0.049, -0.508, -0.082, -1.302, 0.809, -0.01, -0.15,
      -0.035, 0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
      0, 0;
  for (int i = 0; i < getDof(); ++i) {
    path[1][i] = target_q[i];
  }
  target_q << -0.223363, -0.38122, 0.189163, -0.819702, 0.566464, -0.0839337, 0.0170038,
      -0.035,  0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
      0, 0;
  for (int i = 0; i < getDof(); ++i) {
    path[2][i] = target_q[i];
  }
  moveJ(path, 5, false);
}

void executeCommand3() {
  ::Eigen::VectorXd target_qd(getDof());
  target_qd.setZero();
  target_qd[1] = -0.3;
  speedJ(target_qd, 0.5, 0.0);
}

void executeCommand4() {
  speedStop();
}

void executeCommand5() {
  for (int cnt = 0; cnt < 1; ++cnt) {  // 循环执行多少次
    ::std::cout << ">>>>>> cnt = " << cnt << ::std::endl;
    ::Eigen::VectorXd target_q(getDof());
    target_q << -0.049, -0.508, -0.082, -1.302,  0.809, -0.01, -0.15,
        -0.035,  0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
        0, 0;
    moveJ(target_q, 2, 5.4, false);
    ::std::vector<::Eigen::Matrix<double, 6, 1>> ee_pose(2);
    ee_pose = getForwardKinematics();
    ::std::cout << "left ee pose: " << ee_pose[0].transpose() << ::std::endl;
    ::std::cout << "right ee pose: " << ee_pose[1].transpose() << ::std::endl;
    ::std::vector<::Eigen::Matrix<double, 6, 1>> target_ee_pose(2);
    for (int i = 0; i < 2; ++i) {
      target_ee_pose[i] = ee_pose[i];
      target_ee_pose[i][2] += 0.05;
      target_ee_pose[i][3] += 0.3;
    }
    moveL(target_ee_pose, 0.1, 1, false);
    ee_pose = getForwardKinematics();
    ::std::cout << "left ee pose: " << ee_pose[0].transpose() << ::std::endl;
    ::std::cout << "right ee pose: " << ee_pose[1].transpose() << ::std::endl;
  }
}

// MOVEL PATH
void executeCommand6() {
  for (int cnt = 0; cnt < 3; ++cnt) {  // 循环执行多少次
    ::std::cout << ">>>>>> cnt = " << cnt << ::std::endl;

    // moveJ home
    ::Eigen::VectorXd target_q(getDof());
    target_q << -0.049, -0.508, -0.082, -1.302,  0.809, -0.01, -0.15,
        -0.035,  0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
        0, 0;
    moveJ(target_q, 2, 5.4, false);
    ::std::vector<::Eigen::Matrix<double, 6, 1>> ee_pose(2);
    ee_pose = getForwardKinematics();
    ::std::cout << "left ee pose: " << ee_pose[0].transpose() << ::std::endl;
    ::std::cout << "right ee pose: " << ee_pose[1].transpose() << ::std::endl;

    // moveL path
    ::std::vector<::std::vector<double>> target_ee_pose(2, ::std::vector<double>(12, 0));

    for (int i = 0; i < target_ee_pose.size(); ++i) {
      for (int j = 0; j < 12; ++j) {
        target_ee_pose[i][j] = ee_pose[j/6][j%6];
      }
    }

    target_ee_pose[0][2] += 0.05;
    target_ee_pose[1][2] += 0.1;
//    target_ee_pose[2][2] += 0.03;
//    target_ee_pose[3][2] = ee_pose[0][2];

    // 姿态顺序是ZYX
    target_ee_pose[0][10] += 0.3;
    target_ee_pose[1][10] -= 0.3;
//    target_ee_pose[2][9] -= 0.3;
//    target_ee_pose[2][9] = ee_pose[1][5];


    moveL(target_ee_pose, target_ee_pose.size(), false);
  }
}

void executeCommand7() {
  ::std::vector<::Eigen::Matrix<double, 6, 1>> target_xd(2);
  target_xd[0].setZero();
  target_xd[0][3] = 0.3;
  target_xd[1].setZero();
  target_xd[1][2] = 0.03;
  speedL(target_xd, 0.5, 0.0);
}

void executeCommand8() {
  // moveJ home
  ::Eigen::VectorXd target_q(getDof());
  target_q << -0.63267122, -0.90872785, 0.608415, -1.64891, -1.057619, 0.144674, -0.5619749,
      -0.035,  0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
      0, 0;
  moveJ(target_q, 2, 5.4, false);
  ::std::vector<::Eigen::Matrix<double, 6, 1>> ee_pose(2);
  ee_pose = getForwardKinematics();
  ::std::cout << "left ee pose: " << ee_pose[0].transpose() << ::std::endl;
  ::std::cout << "right ee pose: " << ee_pose[1].transpose() << ::std::endl;

  // moveL path
  ::std::vector<::std::vector<double>> right_target_ee_pose =
  {{0.38903604, -0.30935242,  0.17104938,  1.90521488, -1.47887397, -1.99480672},
          {0.38999839, -0.30808249,  0.16170752,  1.97930976, -1.46556088, -2.06553883},
          {0.39030678, -0.308427,    0.158791,    1.90232503, -1.47771524, -1.99169754},
          {0.39315174, -0.30811915,  0.15417429,  1.82091937, -1.48026492, -1.91685577},
          {0.39388516, -0.31152227,  0.14974167,  2.05652995, -1.46479571, -2.11758258},
          {0.39486027, -0.31068297,  0.14636898,  1.98031843, -1.45702271, -2.03858513},
          {0.4010894,  -0.31910656,  0.14236526,  1.98119951, -1.48036218, -1.96372399},
          {0.40056229, -0.31831005,  0.1370883,   2.11459541, -1.46940916, -2.10465159},
          {0.40431381, -0.32369173,  0.13024209,  2.24377129, -1.45678586, -2.17951256},
          {0.40492251, -0.32219126,  0.12684182,  2.0086582,  -1.48716356, -1.96295559},
          {0.40246979, -0.31898494,  0.12087692,  2.1635979,  -1.46282349, -2.13748476},
          {0.40506362, -0.32257035,  0.12348897,  2.19510856, -1.45998263, -2.13723338},
          {0.40503515, -0.32591086,  0.13398327,  2.29662187, -1.46967566, -2.21119876},
          {0.40469488, -0.32742408,  0.13936959,  2.36753249, -1.45003088, -2.26008026},
          {0.3997794,  -0.3205052,   0.15188391,  2.11103464, -1.47635293, -2.07709228},
          {0.39231544, -0.31180064,  0.16057414,  2.07207072, -1.47322969, -2.11917376},
          {0.38958972, -0.30857554,  0.16924027,  1.91328647, -1.45510839, -1.97505958},
          {0.38603752, -0.30753148,  0.17800959,  1.94754664, -1.45584227, -2.01649219},
          {0.38160461, -0.30739536,  0.17947433,  2.2420264,  -1.42807021, -2.31834297},
          {0.37881875, -0.30400638,  0.19135036,  2.27906078, -1.45049632, -2.38456751},
          {0.45373414, -0.07102384,  0.31603847,  1.25086828, -1.12752024, -0.7035869 },
          {0.45373789, -0.06944051,  0.31588126,  1.23895452, -1.12395278, -0.69526068},
          {0.45118142, -0.06707263,  0.31752084,  1.2273271,  -1.11241318, -0.70525528},
          {0.45119398, -0.06008463,  0.30881124,  1.18185972, -1.10302371, -0.65527209},
          {0.45352694, -0.06216935,  0.29491713,  1.1486828,  -1.05389761, -0.62385213},
          {0.46547987, -0.06920672,  0.28548761,  1.0765038,  -1.05194631, -0.46544709},
          {0.465014,   -0.06912575,  0.26986335,  1.14225848, -1.04437314, -0.55549913},
          {0.45864471, -0.07001144,  0.25983927,  1.24225913, -1.03106991, -0.62043826},
          {0.45703112, -0.07286179,  0.25049023,  1.2578337,  -1.00072721, -0.64037425},
          {0.4706771,  -0.06961409,  0.24581937,  1.20671003, -0.93742449, -0.47395536},
          {0.47571874, -0.05173736,  0.2468045,   1.06993242, -0.85945422, -0.45495231},
          {0.47420927, -0.04542226,  0.24421856,  1.17530475, -0.92976981, -0.5650466 },
          {0.48103333, -0.03824537,  0.24270128,  1.1580657,  -0.97751155, -0.53136209},
          {0.4866819,  -0.03051578,  0.24390859,  1.17558355, -0.96099439, -0.5411486 },
          {0.47561058, -0.0175448,   0.24438475,  1.19429264, -0.97835131, -0.66183616},
          {0.46353073, -0.02000932,  0.24515443,  1.26467014, -1.05778023, -0.80428338},
          {0.45795204, -0.02740428,  0.24634378,  1.33456081, -1.12447057, -0.91835186},
          {0.45942605, -0.02184786,  0.26426732,  1.2076797,  -1.12190553, -0.823893  },
          {0.46451559, -0.02358089,  0.28364787,  1.16680917, -1.12260982, -0.79372614},
          {0.46906487, -0.0305704,   0.29026702,  1.15489736, -1.15567176, -0.72661238}};
  ::std::vector<::std::vector<double>> target_ee_pose(right_target_ee_pose.size(), ::std::vector<double>(12, 0));
  for (int i = 0; i < target_ee_pose.size(); ++i) {
    for (int j = 0; j < 6; ++j) {
      target_ee_pose[i][j] = ee_pose[0][j];
      target_ee_pose[i][j+6] = right_target_ee_pose[i][j];
      if (j == 2) {
        target_ee_pose[i][j+6] += 0.5;
      }
    }
  }

  moveL(target_ee_pose, target_ee_pose.size()*0.1, false);
  ::std::cout << "current joints: " << getJointActualPositions().transpose() << ::std::endl;
}

void executeCommand9() {
  ::std::vector<::Eigen::Matrix<double, 6, 1>> ee_pose(2);
  ee_pose = getForwardKinematics();
  for (auto &pose : ee_pose) {
    pose[2] -= 0.5;
  }
  moveNullSpace(ee_pose);
}

void executeCommand10() {
  ::Eigen::VectorXd target_q(getDof());
//  target_q << -0.223363, -0.38122, 0.189163, -0.819702, 0.566464, -0.0839337, 0.0170038,
  target_q << -1.64922361, -0.8093484,  -1.92247498, -1.04525815,  2.69438862, -1.01144587,
                  -0.76254106,
//      -0.0404557 ,
//      target_q << -0.218355,   -0.537118,   -2.84844765, -0.760192,   -2.81455765, -0.318635, -0.0404557,
      -0.035,  0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
      0, 0;
//  target_q << -0.23440392, -0.6990208,   3.03272685, -0.74057585, -2.37160594, -0.22057066,   -0.167488,
//                  -0.035,  0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
//      0, 0;
  //  target_q.setZero();
  //  target_q[0] = -0.049;
  moveJ(target_q, 3, 1.4, false);
}

void executeCommand11() {
  ::Eigen::VectorXd target_q(getDof());
  //  target_q << -0.223363, -0.38122, 0.189163, -0.819702, 0.566464, -0.0839337, 0.0170038,
  target_q << -0.61431086, -1.23449931, -0.3339046,  -1.04525815,  0.44720403,  0.02734781,
                  -0.76254106,
      //      -0.0404557 ,
      //      target_q << -0.218355,   -0.537118,   -2.84844765, -0.760192,   -2.81455765, -0.318635, -0.0404557,
      -0.035,  0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
      0, 0;
  //  target_q << -0.23440392, -0.6990208,   3.03272685, -0.74057585, -2.37160594, -0.22057066,   -0.167488,
  //                  -0.035,  0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
  //      0, 0;
  //  target_q.setZero();
  //  target_q[0] = -0.049;
  moveJ(target_q, 3, 1.4, false);
}

void executeCommand12() {
  for (int cnt = 0; cnt < 1; ++cnt) {  // 循环执行多少次
    ::std::cout << ">>>>>> cnt = " << cnt << ::std::endl;
    ::Eigen::VectorXd target_q(getDof());
    target_q << -0.049, -0.508, -0.082, -1.302,  0.809, -0.01, -0.15,
        -0.035,  0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
        0, 0;
    moveJ(target_q, 2, 5.4, false);
    ::std::vector<::Eigen::Matrix<double, 6, 1>> ee_pose(2);
    ee_pose = getForwardKinematics();
    ::std::cout << "left ee pose: " << ee_pose[0].transpose() << ::std::endl;
    ::std::cout << "right ee pose: " << ee_pose[1].transpose() << ::std::endl;
    ::std::vector<::Eigen::Matrix<double, 6, 1>> target_ee_pose(2);
    for (int i = 0; i < 2; ++i) {
      target_ee_pose[i] = ee_pose[i];
      target_ee_pose[i][2] += 0.05;
      target_ee_pose[i][3] += 0.3;
    }
    moveL(target_ee_pose, target_q, 0.1, 1, false);
    ee_pose = getForwardKinematics();
    ::std::cout << "left ee pose: " << ee_pose[0].transpose() << ::std::endl;
    ::std::cout << "right ee pose: " << ee_pose[1].transpose() << ::std::endl;
  }
}

// 刷新线程函数
void rtThread() {
  const char* user = getenv("USER");
  // 定义基础路径
  ::std::string basePath = "/home/";
  ::raisim::World::setActivationKey(basePath + user + "/.raisim/activation.raisim");
  ::raisim::World world;
  world.setTimeStep(0.001); // 这里需要用数字才能正常在raisimUnity中显示，不然就会显示失败
//  world.addGround();  // 添加地面
  ::std::string urdf_path = basePath + user + "/Documents/example_code/robot_models/URDF-H1_Pro/urdf/URDF-A1_Pro_no_collision.urdf";
  auto nav = world.addArticulatedSystem(urdf_path);
  int dof_raisim = nav->getDOF();
  ::std::cout  << "dof in raisim            : " << dof_raisim << ::std::endl;
  ::std::cout  << "dof of velocity in raisim: " << nav->getGeneralizedVelocityDim() << ::std::endl;
  ::std::cout  << "dof in controller        : " << getDof() << ::std::endl;
  ::std::cout << nav->getBodyNames()[0] << ::std::endl;
  ::std::cout << nav->getBodyNames()[7] << ::std::endl;
  ::std::cout << nav->getBodyNames()[14] << ::std::endl;
  ::std::cout << "Raisim joint order:" << ::std::endl;
  for (int i = 0; i < dof_raisim; ++i) {
    ::std::cout << "Joint[" << i << "] name: " << nav->getMovableJointNames()[i] << ::std::endl;
  }

  ::Eigen::VectorXd q_home(dof_raisim), qd_home(dof_raisim);
  q_home.setZero();
  qd_home.setZero();
  ::Eigen::VectorXd Kp(dof_raisim), Kd(dof_raisim);
  Kp = 100000 * Kp.setOnes();
  Kd = 1000 * Kd.setOnes();
  nav->setGeneralizedCoordinate(q_home);
  nav->setGeneralizedVelocity(qd_home);
  //  nav->setGeneralizedForce(Eigen::VectorXd::Zero(nav->getDOF()));
  //  nav->setPdGains(Kp, Kd);
  ::raisim::RaisimServer server(&world);
  server.launchServer();
  server.focusOn(nav);

  UPLIMB_INPUT_INFO_STRUCT measured_info = {};
  UPLIMB_OUTPUT_INFO_STRUCT command_info = {};
  int sim_cnt = 0;

  while (keepRunning) {
    // raisim设置
    ::std::this_thread::sleep_for(std::chrono::microseconds(1000));
    server.integrateWorldThreadSafe();

    // 获取raisim数据
    auto q_raisim=nav->getGeneralizedCoordinate();
    auto qd_raisim = nav->getGeneralizedVelocity();
    auto qdd_raisim = nav->getGeneralizedAcceleration();
    for (int i = 0; i < dof_raisim; ++i) {
      measured_info.act_pos[i] = q_raisim[i];
      measured_info.act_vel[i] = qd_raisim[i];
      measured_info.act_acc[i] = qdd_raisim[i];
    }

    // 仿真和实物调用相同的接口
    getDriverInfo(&measured_info);
    sendDriverInfo(&command_info);
    if (sim_cnt > 1000) {
      //      ::std::cout << measured_info.act_pos[0] << ::std::endl;
      //      ::std::cout << command_info.cmd_pos[0] << ::std::endl;
      sim_cnt = 0;
    }
    sim_cnt++;

    // 更新仿真数据
    for (int i = 0; i < dof_raisim; ++i) {
      q_raisim[i] = command_info.cmd_pos[i];
      qd_raisim[i] = command_info.cmd_vel[i];
    }

    nav->setGeneralizedCoordinate(q_raisim);
    nav->setGeneralizedVelocity(qd_raisim);
    //    nav->setComputeInverseDynamics(true);
    //    nav->setGeneralizedForce(nav->getGeneralizedForce());
    //    nav->setPdTarget(q_raisim, qd_raisim);

  }
  server.killServer();
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
      { "7", executeCommand7},
      { "8", executeCommand8},
      { "9", executeCommand9},
      { "10", executeCommand10},
      { "11", executeCommand11},
      { "12", executeCommand12},
      { "q", [&]() { keepRunning = false; }},
  };
  ::std::string input;
  while (keepRunning) {
    ::std::cout << "Press Enter to trigger an event" << ::std::endl
                << "0: moveJ_zero\t 1: moveJ_home\t 2: moveJ_path\t 3: speedJ\t 4: speedStop\t "
                << "5: moveL\t 6: moveL_path \t 7: speedL" << ::std::endl
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
  // 启动刷新线程
  ::std::thread rt(rtThread);
  // 启动按键线程
  ::std::thread keyPress(keyPressThread);
  // 启动plot线程
  int dof = getDof();
  ::std::thread plot(zmq_publisher, &keepRunning, &dof);

  // 等待两个线程结束
  rt.join();
  keyPress.join();
  plot.join();

  ::std::cout << ">>>>> Program terminated."  << ::std::endl;
  return 0;
}