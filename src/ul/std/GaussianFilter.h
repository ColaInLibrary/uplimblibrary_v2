/**
 ******************************************************************************
 * @Description   :
 * @author        : Yuan Hanqing
 * @Date          : 25-7-30
 * @Version       : 0.0.1
 * @File          : GaussianFilter.h
 ******************************************************************************
 */
#ifndef UL_SRC_UL_STD_GAUSSIANFILTER_H_
#define UL_SRC_UL_STD_GAUSSIANFILTER_H_
#include <deque>
#include <vector>
#include <iostream>
#include <ul/math/Vector.h>

namespace ul {
namespace std17 {

class GaussianFilter {
private:
    int window_size;
    int window_size_max = 101;
    double sigma;
    int dimension;
    bool initialized;
    std::vector<std::deque<double>> data_buffers; // 每维一个缓存
    Eigen::VectorXd kernel;
    Eigen::VectorXd causalKernel;
    Eigen::VectorXd createKernel(int size, double sigma);

public:
    GaussianFilter(int dimension, int window_size = 21, double sigma = 3.0);
    ~GaussianFilter();
    Eigen::VectorXd update(const Eigen::VectorXd& new_values);

    // 重设滤波参数
    bool resetParas(double sigma, int window_size);
};
}
}
#endif  // UL_SRC_UL_STD_GAUSSIANFILTER_H_
