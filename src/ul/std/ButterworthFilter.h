/**
 ******************************************************************************
 * @Description   :
 * @author        : Yuan Hanqing
 * @Date          : 25-8-7
 * @Version       : 0.0.1
 * @File          : ButterworthFilter.h
 ******************************************************************************
 */
#ifndef UL_SRC_UL_BUTTERWORTHFILTER_H_
#define UL_SRC_UL_BUTTERWORTHFILTER_H_
#include <deque>
#include <vector>
#include <iostream>
#include <ul/math/Vector.h>

namespace ul {
namespace std17 {

class ButterworthFilter {
private:
    int dimension;
    double dt;
    double cutoff;
    bool initialized;

    struct FilterState {
        double x1 = 0, x2 = 0; // 前两次输入
        double y1 = 0, y2 = 0; // 前两次输出
    };

    std::vector<FilterState> states;

    // 滤波器系数
    double a1, a2, b0, b1, b2;

    void computeCoefficients();

public:
    ButterworthFilter(int dimension, double dt = 0.001, double cutoff = 1.0);
    ~ButterworthFilter();

    Eigen::VectorXd update(const Eigen::VectorXd& new_values);
    bool resetParameters(double new_cutoff, double new_dt);
};
}
}
#endif  // UL_SRC_UL_STD_GAUSSIANFILTER_H_
