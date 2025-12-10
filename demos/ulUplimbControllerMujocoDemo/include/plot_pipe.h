#ifndef __PLOT_CONTROLLER_H_
#define __PLOT_CONTROLLER_H_
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <zmq.h>
#include <sched.h>
#include <errno.h>
#include <sys/time.h>
#include <ul/pack/UplimbController.h>

#include <signal.h>
#include <chrono>
#include <mutex>
#include <queue>
#include <cmath>
#include <cstring>
#include "json.hpp"
#include <memory>

#define SLEEP_DURATION 200
#define DATA_MAX_NUM 17
#define TCP_ENDPOINT "tcp://*:9873"
#define TOPIC "test1"
using json = nlohmann::json;

class SharedVectorBuffer {
public:
    SharedVectorBuffer() {
        last_read_data = std::make_shared<Eigen::VectorXd>(Eigen::VectorXd::Zero(7));  // 默认初始化
    }

    // 发送方调用：更新数据
    void set(const Eigen::VectorXd& vec) {
        auto ptr = std::make_shared<Eigen::VectorXd>(vec);
        std::lock_guard<std::mutex> lock(mutex_);
        latest_data_ = ptr;
    }

    // 接收方调用：获取数据，如果有更新则返回新数据，否则返回旧数据
    std::shared_ptr<Eigen::VectorXd> get() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_data_) {
            last_read_data = latest_data_;
            latest_data_.reset();  // 清除新数据标记
        }
        return last_read_data;
    }

private:
    std::shared_ptr<Eigen::VectorXd> latest_data_;     // 写入的最新数据（待读取）
    std::shared_ptr<Eigen::VectorXd> last_read_data;   // 上一次读取的数据（不为空）
    std::mutex mutex_;
};

SharedVectorBuffer shared_buffer;

void generate_data(
    const UPLIMB_INPUT_INFO_STRUCT& stateInfo,
    const UPLIMB_OUTPUT_INFO_STRUCT& cmdInfo,
    const UPLIMB_VAR_OBSERVE& varInfo,
    const Eigen::VectorXd& act_ee_vel_r,
    const Eigen::VectorXd& act_ee_vel_l,
    const Eigen::VectorXd& act_ee_pose_r,
    const Eigen::VectorXd& act_ee_pose_l,
    const Eigen::VectorXd& data,
    char *buffer,
    size_t buffer_size)
{
    json j;
    auto to_vec_d = [](const double* arr) {
        return std::vector<double>(arr, arr + DATA_MAX_NUM);
    };
    auto to_vec_u8 = [](const uint8_t* arr) {
        return std::vector<uint8_t>(arr, arr + DATA_MAX_NUM);
    };
    auto to_vec_u16 = [](const uint16_t* arr) {
        return std::vector<uint16_t>(arr, arr + DATA_MAX_NUM);
    };
    auto to_vec_i = [](const int* arr) {
        return std::vector<int>(arr, arr + DATA_MAX_NUM);
    };

    auto eigen_to_std = [](const Eigen::VectorXd& eigen_vec) {
        if (eigen_vec.size() == 0) return std::vector<double>{};
        return std::vector<double>(eigen_vec.data(), eigen_vec.data() + eigen_vec.size());
    };

    auto concat_vecs = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
        std::vector<double> result;
        result.reserve(a.size() + b.size());
        result.insert(result.end(), a.data(), a.data() + a.size());
        result.insert(result.end(), b.data(), b.data() + b.size());
        return result;
    };

    j["time"] = cmdInfo.downTime/1e9*1.0;
    j["time_up"] = stateInfo.upTime/1e3*1.0;

    j["stateInfo"]["act_pos"] = to_vec_d(stateInfo.act_pos);
    j["stateInfo"]["act_vel"] = to_vec_d(stateInfo.act_vel);
    j["stateInfo"]["act_acc"] = to_vec_d(stateInfo.act_acc);
    j["stateInfo"]["act_tau"] = to_vec_d(stateInfo.act_tau);
    j["stateInfo"]["mode"] = to_vec_u8(stateInfo.mode);
    j["stateInfo"]["statuswd"] = to_vec_u16(stateInfo.statuswd);
    j["stateInfo"]["act_ee_vel"] = concat_vecs(act_ee_vel_l, act_ee_vel_r);
    j["stateInfo"]["act_ee_pose"] = concat_vecs(act_ee_pose_l, act_ee_pose_r);

    j["cmdInfo"]["cmd_pos"] = to_vec_d(cmdInfo.cmd_pos);
    j["cmdInfo"]["cmd_vel"] = to_vec_d(cmdInfo.cmd_vel);
    j["cmdInfo"]["cmd_tau"] = to_vec_d(cmdInfo.cmd_tau);
    j["cmdInfo"]["offset_vel"] = to_vec_d(cmdInfo.offset_vel);
    j["cmdInfo"]["offset_torque"] = to_vec_d(cmdInfo.offset_torque);
    j["cmdInfo"]["mode_operation"] = to_vec_u8(cmdInfo.mode_operation);
    j["cmdInfo"]["ctrlwd"] = to_vec_u16(cmdInfo.ctrlwd);

    j["var"]["var1"] = to_vec_i(varInfo.var1);
    // j["var"]["var2"] = to_vec_i(varInfo.var2);
    // j["var"]["var3"] = to_vec_i(varInfo.var3);
    // j["var"]["var4"] = to_vec_i(varInfo.var4);
    // j["var"]["var5"] = to_vec_i(varInfo.var5);
    j["var"]["var6"] = to_vec_d(varInfo.var6);
    j["var"]["var7"] = to_vec_d(varInfo.var7);
    j["var"]["var8"] = to_vec_d(varInfo.var8);
    j["var"]["var9"] = to_vec_d(varInfo.var9);
    j["var"]["var10"] = to_vec_d(varInfo.var10);

    j["data"]["d1"] = eigen_to_std(data);

    // 序列化为字符串并写入 buffer
    std::string output = j.dump();
    if (buffer_size > 0) {
        std::strncpy(buffer, output.c_str(), buffer_size - 1);
        buffer[buffer_size - 1] = '\0';  // 保证结尾为 null 字符
    }
}


void* zmq_publisher(::std::atomic<bool>* keep_running) {
    std::string input;

    UPLIMB_INPUT_INFO_STRUCT stateInfo;
    UPLIMB_INPUT_INFO_STRUCT stateInfo_t;
    UPLIMB_OUTPUT_INFO_STRUCT cmdInfo;
    UPLIMB_OUTPUT_INFO_STRUCT cmdInfo_t;
    UPLIMB_VAR_OBSERVE varInfo;

    Eigen::VectorXd act_ee_vel_r(6), act_ee_vel_l(6);
    Eigen::VectorXd act_ee_pose_r(6), act_ee_pose_l(6);

    Eigen::VectorXd gravity;

    unsigned long long tp_up, tp_down;

    int zmq_state = 0;
    double ticks = 0;
    void *context = zmq_ctx_new();
    void *publisher = zmq_socket(context, ZMQ_PUB);
    int rc = zmq_bind(publisher, TCP_ENDPOINT); // Bind to TCP endpoint
    if (rc != 0) {
        perror("zmq_bind");
        return NULL;
    }

    while (*keep_running) {
      char buffer[8192*2];
        getArmState(&stateInfo);
        // robotArmIddp->getArmCmd(cmdInfo);
        if (tp_up != stateInfo.upTime) {
          std::cout << "HHHHHHHHHHH" << std::endl;
        // if (tp_down != cmdInfo.downTime){
            // shared_buffer.set(getJointActualPositions(14));
            auto data = shared_buffer.get();

            getArmCMD(&cmdInfo);
//            getArmVar(&varInfo);
            // robotArmIddp->getArmState(stateInfo);
//            getArmVar(varInfo);

//            act_ee_vel_r = getEEVel(1);
//            act_ee_vel_l = getEEVel(2);
   
            // getGravityTorque(gravity);
            // shared_buffer.set(gravity);

            // act_ee_vel_r.setZero();
            // act_ee_vel_l.setZero();

            // act_ee_pose_l.setZero();
            // act_ee_pose_r.setZero();

//            act_ee_pose_l = getForwardKinematics(1);
//            act_ee_pose_r = getForwardKinematics(2);
            
            generate_data(stateInfo, cmdInfo, varInfo, act_ee_vel_r, act_ee_vel_l, act_ee_pose_r, act_ee_pose_l, *data, buffer, sizeof(buffer));
        
            zmq_send(publisher, TOPIC, strlen(TOPIC), ZMQ_SNDMORE);
            zmq_send(publisher, buffer, strlen(buffer), 0);
            
            tp_up = stateInfo.upTime;
            // tp_down = cmdInfo.downTime;
            ticks += 0.001;
        }
        usleep(SLEEP_DURATION);
    }
    std::cout << "===== Plot Task exiting... =====" << std::endl;
    zmq_close(publisher);
    zmq_ctx_destroy(context);
    return NULL;
}

// void* key_handling(void* RobotArmIddp) {
//     RobotIddp *robotArmIddp = static_cast<RobotIddp*>(RobotArmIddp);
//     std::string input;
//     std::string controller_paras_file_l = "/home/nav01/CodeFiles/xenomaixddpproject/RobotIddp/config/Uplimb/config/torque_controller_paras_l.yaml";
//     std::string controller_paras_file_r = "/home/nav01/CodeFiles/xenomaixddpproject/RobotIddp/config/Uplimb/config/torque_controller_paras_r.yaml";
//     while (robotArmIddp->run) {
//         std::getline(std::cin, input);
//         if (input == "l") {
//             std::cout << "#########################################" << std::endl;
//             loadTorqueControllerPara(controller_paras_file_l.c_str(), 0);
//             loadTorqueControllerPara(controller_paras_file_r.c_str(), 1);
//             std::cout << "#########################################" << std::endl;
//         }
//         usleep(1000); 
//     }
//     return NULL;
// }


void set_thread_priority(pthread_t thread, int policy, int priority) {
    struct sched_param sch_params;
    sch_params.sched_priority = priority;

    if (pthread_setschedparam(thread, policy, &sch_params)) {
        fprintf(stderr, "Failed to set thread scheduling: %s\n", strerror(errno));
    }
}

#endif
