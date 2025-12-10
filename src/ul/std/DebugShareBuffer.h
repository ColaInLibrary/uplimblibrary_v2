/**
 ******************************************************************************
 * @Description   :
 * @author        : Yuan Hanqing
 * @Date          : 25-8-8
 * @Version       : 0.0.1
 * @File          : DebugShareBuffer.h
 ******************************************************************************
 */
#ifndef UL_SRC_UL_STD_DEBUG_SHARE_BUFFER_H_
#define UL_SRC_UL_STD_DEBUG_SHARE_BUFFER_H_
#include <vector>
#include <mutex>
#include <atomic>
#include <iostream>
#include <ul/math/Vector.h>
#include <ul/std/common.h>

namespace ul {
namespace std17 {
class DebugShareBuffer {
public:
    DebugShareBuffer(size_t size, int dim);

    // 生产者：更新指定索引的数据
    void set(size_t index, const Eigen::VectorXd& vec);

    // 消费者：获取最新的一帧完整数据，返回 true 表示新数据可用
    bool get(std::vector<Eigen::VectorXd>& out);

private:
    std::vector<Eigen::VectorXd> buffers_[2];
    int write_index_;
    int read_index_;
    int dim_;  // 固定维度
    std::mutex mutex_;
    std::atomic<bool> has_new_data_;
};
}
}
#endif  // UL_SRC_UL_STD_DEBUG_SHARE_BUFFER_H_
