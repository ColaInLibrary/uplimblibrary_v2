/**
 ******************************************************************************
 * @Description   :
 * @author        : Yuan Hanqing
 * @Date          : 25-7-30
 * @Version       : 0.0.1
 * @File          : gaussianFilter.cpp
 ******************************************************************************
 */
#include "GaussianFilter.h"

namespace ul {
namespace std17 {
GaussianFilter::GaussianFilter(int dimension, int window_size, double sigma)
    : dimension(dimension), sigma(sigma), initialized(false) {
    if (window_size % 2 == 0) {
        window_size += 1;
    }
    this->window_size = window_size;
    kernel = createKernel(window_size, sigma);
    data_buffers.resize(dimension);
}

GaussianFilter::~GaussianFilter() {}

Eigen::VectorXd GaussianFilter::update(const Eigen::VectorXd& new_values) {
    Eigen::VectorXd smoothed_values = Eigen::VectorXd::Zero(dimension);
    if (new_values.size() != dimension) {
        std::cerr << "Dimension mismatch in update!" << std::endl;
        return new_values;
    }

    if (!initialized) {
        for (int d = 0; d < dimension; ++d) {
            for (int i = 0; i < window_size_max; ++i) {
                data_buffers[d].push_back(0.0);
            }
        }
        initialized = true;
    } else {
        for (int d = 0; d < dimension; ++d) {
            data_buffers[d].push_back(new_values[d]);
            if ((int)data_buffers[d].size() > window_size_max) {
                data_buffers[d].pop_front();
            }
        }
    }

    int kSize = causalKernel.size();
    for (int d = 0; d < dimension; ++d) {
        if ((int)data_buffers[d].size() < kSize) {
            smoothed_values[d] = new_values[d];
        } else {
            for (int i = 0; i < kSize; ++i) {
                smoothed_values[d] += data_buffers[d][data_buffers[d].size() - kSize + i] * causalKernel[kSize - 1 - i];
            }
        }
    }

    return smoothed_values;
}

Eigen::VectorXd GaussianFilter::createKernel(int size, double sigma) {
    Eigen::VectorXd kernel_full = Eigen::VectorXd::Zero(size);
    double sum = 0.0;
    double center = (size - 1) / 2.0;
    
    for (int i = 0; i < size; ++i) {
        kernel_full[i] = std::exp(-0.5 * std::pow((i - center) / sigma, 2));
        sum += kernel_full[i];
    }
    kernel_full /= sum;

    int start = size / 2;
    Eigen::VectorXd causal = kernel_full.segment(start, size - start);
    double causalSum = causal.sum();
    causal /= causalSum;
    causalKernel = causal;
    
    return kernel_full;
}

bool GaussianFilter::resetParas(double sigma, int window_size) {
    if (window_size % 2 == 0) {
        window_size += 1;
    }

    if (window_size > 2 * window_size_max - 1 || window_size < 3) {
        std::cerr << "GaussianFilter: parameter window_size out of the limit !" << std::endl;
        return false;
    }
    
    if (window_size == this->window_size && sigma == this->sigma) {
        return true;
    } else {
        this->window_size = window_size;
        kernel = createKernel(window_size, sigma);
        return true;
    } 
}
}
}
