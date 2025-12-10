/**
 ******************************************************************************
 * @Description   :
 * @author        : Yuan Hanqing
 * @Date          : 25-8-7
 * @Version       : 0.0.1
 * @File          : Differentiator.h
 ******************************************************************************
 */
#ifndef UL_SRC_UL_STD_DIFFERENTIATOR_H_
#define UL_SRC_UL_STD_DIFFERENTIATOR_H_
#include <deque>
#include <vector>
#include <iostream>
#include <ul/math/Vector.h>

namespace ul {
namespace std17 {

class Differentiator {
private:
    int dimension;
    double dt;
    int max_order;
    bool initialized;

    std::vector<std::deque<double>> data_buffers;

public:
    Differentiator(int dimension, double dt = 0.001, int max_order = 2);
    ~Differentiator();

    // 修改返回类型
    std::vector<Eigen::VectorXd> update(const Eigen::VectorXd& new_values);

    bool resetDt(double new_dt);
    bool resetOrder(int new_order);
};
}
}
#endif  // UL_SRC_UL_STD_DIFFERENTIATOR_H_
