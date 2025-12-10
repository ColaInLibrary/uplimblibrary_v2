/**
 ******************************************************************************
 * @Description   :
 * @author        : Yuan Hanqing
 * @Date          : 25-5-14
 * @Version       : 0.0.1
 * @File          : ButterworthFilter.cpp
 ******************************************************************************
 */
#include "ButterworthFilter.h"

namespace ul {
namespace std17 {
ButterworthFilter::ButterworthFilter(int dimension, double dt, double cutoff)
    : dimension(dimension), dt(dt), cutoff(cutoff), initialized(false) {
    states.resize(dimension);
    computeCoefficients();
}

ButterworthFilter::~ButterworthFilter() {}

void ButterworthFilter::computeCoefficients() {
    double omega = 2.0 * M_PI * cutoff;
    double tan_omega = std::tan(omega * dt / 2.0);

    double tan_omega_squared = tan_omega * tan_omega;
    double sqrt2 = std::sqrt(2.0);

    double norm = 1.0 / (1.0 + sqrt2 * tan_omega + tan_omega_squared);

    b0 = tan_omega_squared * norm;
    b1 = 2.0 * b0;
    b2 = b0;

    a1 = 2.0 * (tan_omega_squared - 1.0) * norm;
    a2 = (1.0 - sqrt2 * tan_omega + tan_omega_squared) * norm;
}

Eigen::VectorXd ButterworthFilter::update(const Eigen::VectorXd& new_values) {
    Eigen::VectorXd output = Eigen::VectorXd::Zero(dimension);

    if (new_values.size() != dimension) {
        std::cerr << "Dimension mismatch in update!" << std::endl;
        return new_values;
    }

    if (!initialized) {
        for (int d = 0; d < dimension; ++d) {
            states[d].x1 = new_values[d];
            states[d].x2 = new_values[d];
            states[d].y1 = new_values[d];
            states[d].y2 = new_values[d];
        }
        initialized = true;
        return new_values;
    }

    for (int d = 0; d < dimension; ++d) {
        double x0 = new_values[d];

        double y = b0 * x0 + b1 * states[d].x1 + b2 * states[d].x2
                 - a1 * states[d].y1 - a2 * states[d].y2;

        // 更新状态
        states[d].x2 = states[d].x1;
        states[d].x1 = x0;

        states[d].y2 = states[d].y1;
        states[d].y1 = y;

        output[d] = y;
    }

    return output;
}

bool ButterworthFilter::resetParameters(double new_cutoff, double new_dt) {
    if (new_cutoff <= 0.0 || new_dt <= 0.0) {
        std::cerr << "Invalid parameters: cutoff and dt must be positive." << std::endl;
        return false;
    }

    if (new_cutoff == cutoff && new_dt == dt) {
        return true;
    }

    cutoff = new_cutoff;
    dt = new_dt;
    computeCoefficients();
    return true;
}
}
}
