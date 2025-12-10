/**
 ******************************************************************************
 * @Description   :
 * @author        : Yuan Hanqing
 * @Date          : 25-8-8
 * @Version       : 0.0.1
 * @File          : DebugShareBuffer.cpp
 ******************************************************************************
 */
#include "DebugShareBuffer.h"

namespace ul {
namespace std17 {
DebugShareBuffer::DebugShareBuffer(size_t size, int dim)
    : write_index_(0),
      read_index_(1),
      dim_(dim),
      has_new_data_(false)
{
    buffers_[0] = std::vector<Eigen::VectorXd>(size, Eigen::VectorXd::Zero(dim));
    buffers_[1] = std::vector<Eigen::VectorXd>(size, Eigen::VectorXd::Zero(dim));
}

void DebugShareBuffer::set(size_t index, const Eigen::VectorXd& vec) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (index >= buffers_[write_index_].size()) {
        std::cout << YELLOW << "[DebugShareBuffer] Index " << index << " out of range.\n" << RESET << std::endl;
        return;
    }

    Eigen::VectorXd adjusted(dim_);
    if (vec.size() == dim_) {
        adjusted = vec;
    } else if (vec.size() < dim_) {
        adjusted.setZero();
        adjusted.head(vec.size()) = vec;
        // std::cout << YELLOW << "[DebugShareBuffer] Warning: Input vector smaller than dim (" 
        //           << vec.size() << " < " << dim_ << "), zero-padded.\n" << RESET << std::endl;
    } else { // vec.size() > dim_
        adjusted = vec.head(dim_);
        // std::cout << YELLOW << "[DebugShareBuffer] Warning: Input vector larger than dim (" 
        //           << vec.size() << " > " << dim_ << "), truncated.\n" << RESET << std::endl;
    }

    buffers_[write_index_][index] = adjusted;
    has_new_data_.store(true, std::memory_order_release);
}

bool DebugShareBuffer::get(std::vector<Eigen::VectorXd>& out) {
    if (has_new_data_.load(std::memory_order_acquire)) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::swap(write_index_, read_index_);
        out = buffers_[read_index_];  // 返回副本
        has_new_data_.store(false, std::memory_order_release);
        return true;
    }
    return false;
}
}
}
