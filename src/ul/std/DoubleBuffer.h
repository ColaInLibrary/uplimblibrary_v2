/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-11-12
 * @Version       : 0.0.1
 * @File          : DoubleBuffer.h
 ******************************************************************************
 */

#ifndef UL_SRC_UL_STD_DOUBLEBUFFER_H_
#define UL_SRC_UL_STD_DOUBLEBUFFER_H_

#include <Eigen/Dense>
#include <array>
#include <atomic>
#include <iostream>
#include <thread>
#include "Socket.h"

namespace ul {
namespace std17 {

// 非实时侧写、实时侧读的双缓冲区结构
struct N2RBuffer {
  alignas(64) std::array<double, UPLIMB_MAX_DIMENSION> q;    // 关节角度
  alignas(64) std::array<double, UPLIMB_MAX_DIMENSION> qd;   // 关节速度
  alignas(64) std::array<double, UPLIMB_MAX_DIMENSION> qdd;  // 关节加速度
  alignas(64) std::array<double, 18> x;   // 左臂、右臂、头部 笛卡尔位姿
  alignas(64) std::array<double, 18> xd;  // 左臂、右臂、头部 笛卡尔速度
  size_t actual_size{0};
  double speed{0.0};                                  // moveJ中的参数
  double acceleration{0.0};                           // moveJ, speedJ中的参数
  bool asynchronous{false};                           // moveJ中的参数
  int arm_type{0};                                    // moveJ中的参数
  int id{0};                                          // 接收到的指令ID
  InterfaceType cmd_type;                             // 指令类型
  std::atomic<bool> ready{false};
};

template <size_t MAX_DIMENSION> class DoubleBuffer {
private:

  // 实时侧写、非实时侧读的双缓冲区结构
  struct R2NBuffer {
    Eigen::Matrix<double, MAX_DIMENSION, 1> q;     // 实际的关节角度
    Eigen::Matrix<double, MAX_DIMENSION, 1> qd;    // 实际的关节速度
    Eigen::Matrix<double, MAX_DIMENSION, 1> qdd;   // 实际的关节加速度
    Eigen::Matrix<double, MAX_DIMENSION, 1> qddd;  // 实际的关节 jerk
    Eigen::Matrix<double, 6, 1> x_left;            // 左臂笛卡尔实际位姿
    Eigen::Matrix<double, 6, 1> x_right;           // 右臂笛卡尔实际位姿
    Eigen::Matrix<double, 6, 1> x_head;            // 头部 笛卡尔实际位姿
    Eigen::Matrix<double, 6, 1> xd_left;           // 左臂笛卡尔实际速度
    Eigen::Matrix<double, 6, 1> xd_right;          // 右臂笛卡尔实际速度
    Eigen::Matrix<double, 6, 1> xd_head;           // 头部 笛卡尔实际速度
    size_t actual_size{0};
    std::atomic<bool> ready{false};
  };

  // 非实时 -> 实时侧 传输数据
  N2RBuffer n2r_buffers_[2];
  std::atomic<int> n2r_write_index_{0};
  std::atomic<int> n2r_read_index_{1};
//  std::atomic<uint64_t> n2r_write_count_{0};
//  std::atomic<uint64_t> n2r_read_count_{0};

  // 实时 -> 非实时侧 传输数据
  R2NBuffer r2n_buffers_[2];
  std::atomic<int> r2n_write_index_{0};
  std::atomic<int> r2n_read_index_{1};
//  std::atomic<uint64_t> r2n_write_count_{0};
//  std::atomic<uint64_t> r2n_read_count_{0};

public:
  DoubleBuffer() {
    std::cout << "=== 双缓冲系统初始化 ===" << std::endl;
    std::cout << "最大维度: " << MAX_DIMENSION << std::endl;
    std::cout << "=========================" << std::endl;
  }

  // 非实时侧写入
  bool N2RWriteData(const SocketCommand& cmd) {
    int write_idx = n2r_write_index_.load(std::memory_order_relaxed);
    N2RBuffer &write_buf = n2r_buffers_[write_idx];

    // 检查目标缓冲区是否可用
    if (write_buf.ready.load(std::memory_order_acquire)) {
//      std::cout << "⚠️  写入失败: 缓冲区[" << write_idx << "]仍在被读取"
//                << std::endl;
      return false;
    }

    // 命令处理函数
      try {
        write_buf.cmd_type = cmd.cmd;
        write_buf.id = cmd.id;
        write_buf.speed = cmd.params.value("speed", 0.0);
        write_buf.acceleration = cmd.params.value("acceleration", 0.0);
        write_buf.asynchronous = cmd.params.value("asynchronous", false);
        write_buf.arm_type = cmd.params.value("arm_type", 0);

        auto positions = cmd.params.value("q", std::vector<double>(UPLIMB_MAX_DIMENSION, 0.0));
        auto velocities = cmd.params.value("qd", std::vector<double>(UPLIMB_MAX_DIMENSION, 0.0));
        auto accelerations = cmd.params.value("qdd", std::vector<double>(UPLIMB_MAX_DIMENSION, 0.0));
        write_buf.actual_size = positions.size();

        // 打印调试
        std::cout << "Arm type: " << write_buf.arm_type << std::endl;
        std::cout << "Positions: ";
        for (const auto& pos : positions) {
          std::cout << pos << " ";
        }
        std::cout << std::endl;

        if (write_buf.actual_size > MAX_DIMENSION) {
          throw std::runtime_error("Input dimension exceeds MAX_DIMENSION");
        }

        // 使用memcpy可能比std::copy更快
        if (write_buf.actual_size > 0) {
          std::memcpy(write_buf.q.data(), positions.data(),
                      write_buf.actual_size * sizeof(double));
          std::memcpy(write_buf.qd.data(), velocities.data(),
                      write_buf.actual_size * sizeof(double));
          std::memcpy(write_buf.qdd.data(), accelerations.data(),
                      write_buf.actual_size * sizeof(double));
        }
      } catch (const json::exception& e) {
        std::cerr << "MOVEJ参数解析错误: " << e.what() << std::endl;
      }

    // 标记就绪，Release屏障：确保数据写入在ready=true之前完成
    write_buf.ready.store(true, std::memory_order_release);

//    write_buf.printInfo("  写入后", write_idx);

    // 交换缓冲区
    swapBuffers(n2r_write_index_, n2r_read_index_);

    return true;
  }

  // 实时侧读取
  bool N2RReadData(N2RBuffer &cmd_paras) {
    int read_idx = n2r_read_index_.load(std::memory_order_relaxed);
    N2RBuffer &read_buf = n2r_buffers_[read_idx];

    if (!read_buf.ready.load(std::memory_order_acquire)) {
//      std::cout << "⏸️  读取失败: 缓冲区[" << read_idx << "]数据未就绪"
//                << std::endl;
      return false;
    }

    // 手动拷贝数据，避免任何动态行为
    for (int i = 0; i < read_buf.actual_size; ++i) {
      cmd_paras.q[i] = read_buf.q[i];
      cmd_paras.qd[i] = read_buf.qd[i];
      cmd_paras.qdd[i] = read_buf.qdd[i];
    }
    // 对于未使用的元素，可以设为0或保持原值
    for (int i = read_buf.actual_size; i < MAX_DIMENSION; ++i) {
      cmd_paras.q[i] = 0.0;  // 可选：清理未使用部分
      cmd_paras.qd[i] = 0.0;
      cmd_paras.qdd[i] = 0.0;
    }

    cmd_paras.speed = read_buf.speed;
    cmd_paras.acceleration = read_buf.acceleration;
    cmd_paras.asynchronous = read_buf.asynchronous;
    cmd_paras.arm_type = read_buf.arm_type;
    cmd_paras.id = read_buf.id;

    // 重置标志，将ready标志设置为false，表示数据已读取完毕
    read_buf.ready.store(false, std::memory_order_release);
//    n2r_read_count_++;

//    std::cout << "  读取数据: [";
//    for (int i = 0; i < out_q.size(); ++i) {
//      std::cout << out_q[i];
//      if (i < out_q.size() - 1)
//        std::cout << ", ";
//    }
//    std::cout << "]" << std::endl;
//    std::cout << "✅ 读取完成 (总计读取: " << read_count_.load() << " 次)"
//              << std::endl;

    return true;
  }

  // 实时侧写入
  bool R2NWriteData(const Eigen::VectorXd &q, double speed = 1.05,
                    double acceleration = 1.4, bool asynchronous = false,
                    int arm_type = 15) {
    int write_idx = r2n_write_index_.load(std::memory_order_relaxed);
    R2NBuffer &write_buf = r2n_buffers_[write_idx];

    // 检查目标缓冲区是否可用
    if (q.size() > MAX_DIMENSION) {
      return false;
    }

    // 拷贝数据到缓冲区
//    write_buf.actual_size = q.size();
//    for (int i = 0; i < q.size(); ++i) {
//      write_buf.data[i] = q[i];
//    }

    // 标记就绪，Release屏障：确保数据写入在ready=true之前完成
    write_buf.ready.store(true, std::memory_order_release);

    // 交换缓冲区
    swapBuffers(r2n_write_index_, r2n_read_index_);

    return true;
  }

  // 非实时侧读取
  bool R2NReadData(Eigen::Matrix<double, MAX_DIMENSION, 1>& out_q,
                   Eigen::Matrix<double, MAX_DIMENSION, 1>& out_qd,
                   Eigen::Matrix<double, MAX_DIMENSION, 1>& out_qdd,
                   Eigen::Matrix<double, 6, 1>& out_x_left,
                   Eigen::Matrix<double, 6, 1>& out_x_right,
                   Eigen::Matrix<double, 6, 1>& out_x_head) {
    int read_idx = r2n_read_index_.load(std::memory_order_relaxed);
    R2NBuffer &read_buf = r2n_buffers_[read_idx];

    if (!read_buf.ready.load(std::memory_order_acquire)) {
      return false;
    }

    // 手动拷贝数据，避免任何动态行为
    for (int i = 0; i < read_buf.actual_size; ++i) {
      out_q[i] = read_buf.q[i];
    }
    // 对于未使用的元素，可以设为0或保持原值
    for (int i = read_buf.actual_size; i < MAX_DIMENSION; ++i) {
      out_q[i] = 0.0;  // 可选：清理未使用部分
    }

    // 重置标志，将ready标志设置为false，表示数据已读取完毕
    read_buf.ready.store(false, std::memory_order_release);

    return true;
  }

//  void printBufferStatus() {
//    std::cout << "\n📊 当前缓冲区状态:" << std::endl;
//    std::cout << "写索引: " << write_index_.load()
//              << ", 读索引: " << read_index_.load() << std::endl;
//    buffers_[0].printInfo("  缓冲区0", 0);
//    buffers_[1].printInfo("  缓冲区1", 1);
//    std::cout << "----------------------" << std::endl;
//  }

private:
  void swapBuffers(std::atomic<int> &write_index, std::atomic<int> &read_index) {
    int current_write = write_index.load(std::memory_order_relaxed);
    int current_read = read_index.load(std::memory_order_relaxed);

    int new_write = current_read;
    int new_read = current_write;

    write_index.store(new_write, std::memory_order_release);
    read_index.store(new_read, std::memory_order_release);

//    std::cout << "🔄 缓冲区切换: 写索引 " << current_write << "→" << new_write
//              << ", 读索引 " << current_read << "→" << new_read << std::endl;
  }
};
}
}
#endif // UL_SRC_UL_STD_DOUBLEBUFFER_H_
