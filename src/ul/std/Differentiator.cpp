/**
 ******************************************************************************
 * @Description   :
 * @author        : Yuan Hanqing
 * @Date          : 25-5-14
 * @Version       : 0.0.1
 * @File          : Differentiator.cpp
 ******************************************************************************
 */
#include "Differentiator.h"

namespace ul {
namespace std17 {
Differentiator::Differentiator(int dimension, double dt, int max_order)
    : dimension(dimension), dt(dt), max_order(max_order), initialized(false) {
    data_buffers.resize(dimension);
}

Differentiator::~Differentiator() {}

std::vector<Eigen::VectorXd> Differentiator::update(const Eigen::VectorXd& new_values) {
    // 准备每阶导数的结果容器
    std::vector<Eigen::VectorXd> derivatives;
    for (int order = 0; order < max_order; ++order) {
        derivatives.emplace_back(Eigen::VectorXd::Zero(dimension));
    }

    if (new_values.size() != dimension) {
        std::cerr << "Dimension mismatch in update!" << std::endl;
        return derivatives;
    }

    for (int d = 0; d < dimension; ++d) {
        data_buffers[d].push_back(new_values[d]);
        int required_points = 2 * max_order + 1;
        if ((int)data_buffers[d].size() > required_points) {
            data_buffers[d].pop_front();
        }
    }

    for (int d = 0; d < dimension; ++d) {
        int N = data_buffers[d].size();

        if (max_order >= 1 && N >= 3) {
            double fwd = data_buffers[d][N - 1];
            double bwd = data_buffers[d][N - 3];
            derivatives[0][d] = (fwd - bwd) / (2.0 * dt);
        }

        if (max_order >= 2 && N >= 3) {
            double center = data_buffers[d][N - 2];
            double fwd = data_buffers[d][N - 1];
            double bwd = data_buffers[d][N - 3];
            derivatives[1][d] = (fwd - 2 * center + bwd) / (dt * dt);
        }

        // 可拓展三阶导数
    }

    return derivatives;
}

bool Differentiator::resetDt(double new_dt) {
    if (new_dt <= 0.0) {
        std::cerr << "dt must be positive!" << std::endl;
        return false;
    }
    this->dt = new_dt;
    return true;
}

bool Differentiator::resetOrder(int new_order) {
    if (new_order < 1 || new_order > 3) {
        std::cerr << "Supported order: 1 to 3" << std::endl;
        return false;
    }
    this->max_order = new_order;
    return true;
}
}
}
