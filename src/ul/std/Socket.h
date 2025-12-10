/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-11-13
 * @Version       : 0.0.1
 * @File          : Socket.h
 ******************************************************************************
 */

#ifndef UL_SRC_UL_STD_SOCKET_H_
#define UL_SRC_UL_STD_SOCKET_H_

#include <zmq.hpp>
#include <iostream>
#include <string>
#include <unordered_map>
#include <nlohmann/json.hpp>


namespace ul::std17 {

using json = nlohmann::json;

// 指令枚举
enum class InterfaceType {
  MOVEJ,
  MOVEL,
  UNKNOWN,
  // ... 其他指令
};

struct SocketCommand {
  InterfaceType cmd;
  int id{0};
  double timestamp{0};
  json params;
};

class ZmqCommandServer {
private:
  zmq::context_t context_;
  zmq::socket_t socket_;
  std::string endpoint_;

  // 命令类型转换函数
  InterfaceType string_to_command_type(const std::string& cmd_str) {
    static std::unordered_map<std::string, InterfaceType> cmd_map = {
        {"MOVEJ", InterfaceType::MOVEJ},
        {"MOVEL", InterfaceType::MOVEL},
        {"UNKNOWN", InterfaceType::UNKNOWN},
        // ... 添加其他命令映射
    };

    auto it = cmd_map.find(cmd_str);
    if (it != cmd_map.end()) {
      return it->second;
    }
    return InterfaceType::UNKNOWN;
  }

  // 解析JSON指令
  SocketCommand parse_command(const std::string& json_str) {
    try {
      auto j = json::parse(json_str);
      SocketCommand cmd;

      // 安全地解析命令类型
      std::string cmd_str = j["cmd"].get<std::string>();
      std::cout << "Received command: " << cmd_str << std::endl;
      cmd.cmd = string_to_command_type(cmd_str);

      // value()方法的作用: 安全地获取JSON字段，如果字段不存在则返回默认值。
      cmd.id = j.value("id", 0);
      cmd.timestamp = j.value("timestamp", 0.0);
      cmd.params = j.value("params", json::object());

      return cmd;
    } catch (const json::exception& e) {
      std::cerr << "JSON parse error: " << e.what() << std::endl;
      std::cerr << "Org JSON: " << json_str << std::endl;
      throw;
    }
  }

  // 命令处理函数
  void handle_moveJ(const json& params) {
    try {
      auto positions = params["q"].get<std::vector<double>>();
      double velocity = params["speed"];
      double acceleration = params["acceleration"];
      bool asynchronous = params.value("asynchronous", false);
      int arm_type = params["arm_type"];

      // 执行运动指令
      std::cout << "Arm type: " << arm_type << std::endl;
      std::cout << "Positions: ";
      for (const auto& pos : positions) {
        std::cout << pos << " ";
      }
      std::cout << std::endl;
      std::cout << "Velocity: " << velocity << std::endl;
      std::cout << "Acceleration: " << acceleration << std::endl;
      std::cout << "Asynchronous: " << asynchronous << std::endl;

    } catch (const json::exception& e) {
      std::cerr << "MOVEJ参数解析错误: " << e.what() << std::endl;
    }
  }

  void handle_moveL(const json& params) {
    try {
      auto positions = params["positions"].get<std::vector<double>>();
      double velocity = params["velocity"];
      double acceleration = params["acceleration"];
      bool blocking = params.value("blocking", true);

      // 执行运动指令
      std::cout << "MoveL - Positions: ";
      for (const auto& pos : positions) {
        std::cout << pos << " ";
      }
      std::cout << std::endl;
      std::cout << "Velocity: " << velocity << std::endl;
      std::cout << "Acceleration: " << acceleration << std::endl;
      std::cout << "Blocking: " << blocking << std::endl;

    } catch (const json::exception& e) {
      std::cerr << "MOVEL参数解析错误: " << e.what() << std::endl;
    }
  }

  // 处理指令
  void handle_command(const SocketCommand& cmd) {
    switch (cmd.cmd) {
    case InterfaceType::MOVEJ:
      handle_moveJ(cmd.params);
      break;
    case InterfaceType::MOVEL:
      handle_moveL(cmd.params);
      break;
    case InterfaceType::UNKNOWN:
      // TODO: 实现MOVEC处理
      std::cout << "UNKNOWN command received" << std::endl;
      break;
    default:
      std::cerr << "未知命令类型" << std::endl;
      break;
    }
  }

public:
  // 构造函数
  ZmqCommandServer(const std::string& endpoint, int zmq_type = ZMQ_REP)
      : context_(1), socket_(context_, zmq_type), endpoint_(endpoint) {

    try {
      socket_.bind(endpoint_);
      std::cout << "ZMQ server starting: " << endpoint_ << std::endl;
    } catch (const zmq::error_t& e) {
      std::cerr << "ZMQ binding error: " << e.what() << std::endl;
      throw;
    }
  }

  // 析构函数
  ~ZmqCommandServer() {
    socket_.close();
    context_.close();
  }

  // 接收并处理消息
  bool receive(SocketCommand &cmd) {
    try {
      zmq::message_t request;

      // 等待接收消息
      auto result = socket_.recv(request, zmq::recv_flags::dontwait);
      if (!result) {
        return false; // 没有消息可接收
      }

      std::string message_str(static_cast<char*>(request.data()), request.size());
      std::cout << "收到消息: " << message_str << std::endl;

      // 解析命令
      cmd = parse_command(message_str);

      // 发送响应
      json response = {
          {"status", "success"},
          {"id", cmd.id},
          {"message", "SocketCommand processed successfully"}
      };

      std::string response_str = response.dump();
      zmq::message_t reply(response_str.size());
      memcpy(reply.data(), response_str.c_str(), response_str.size());
      socket_.send(reply, zmq::send_flags::none);

      return true;

    } catch (const std::exception& e) {
      std::cerr << "处理消息时发生错误: " << e.what() << std::endl;

      // 发送错误响应
      json error_response = {
          {"status", "error"},
          {"message", e.what()}
      };

      std::string error_str = error_response.dump();
      zmq::message_t error_reply(error_str.size());
      memcpy(error_reply.data(), error_str.c_str(), error_str.size());
      socket_.send(error_reply, zmq::send_flags::none);

      return false;
    }
  }


  // 接收并处理消息
  bool receive_and_handle() {
    try {
      zmq::message_t request;

      // 等待接收消息
      auto result = socket_.recv(request, zmq::recv_flags::dontwait);
      if (!result) {
        return false; // 没有消息可接收
      }

      std::string message_str(static_cast<char*>(request.data()), request.size());
      std::cout << "收到消息: " << message_str << std::endl;

      // 解析命令
      SocketCommand cmd = parse_command(message_str);

      // 处理命令
      handle_command(cmd);

      // 发送响应
      json response = {
          {"status", "success"},
          {"id", cmd.id},
          {"message", "SocketCommand processed successfully"}
      };

      std::string response_str = response.dump();
      zmq::message_t reply(response_str.size());
      memcpy(reply.data(), response_str.c_str(), response_str.size());
      socket_.send(reply, zmq::send_flags::none);

      return true;

    } catch (const std::exception& e) {
      std::cerr << "处理消息时发生错误: " << e.what() << std::endl;

      // 发送错误响应
      json error_response = {
          {"status", "error"},
          {"message", e.what()}
      };

      std::string error_str = error_response.dump();
      zmq::message_t error_reply(error_str.size());
      memcpy(error_reply.data(), error_str.c_str(), error_str.size());
      socket_.send(error_reply, zmq::send_flags::none);

      return false;
    }
  }

  // 获取端点地址
  std::string get_endpoint() const {
    return endpoint_;
  }

  // 设置接收超时（毫秒）
  void set_receive_timeout(int timeout_ms) {
    socket_.setsockopt(ZMQ_RCVTIMEO, &timeout_ms, sizeof(timeout_ms));
  }

  // 设置发送超时（毫秒）
  void set_send_timeout(int timeout_ms) {
    socket_.setsockopt(ZMQ_SNDTIMEO, &timeout_ms, sizeof(timeout_ms));
  }
};

}

#endif // UL_SRC_UL_STD_SOCKET_H_
