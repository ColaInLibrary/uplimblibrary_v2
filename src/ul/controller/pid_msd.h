/*
 * @Description:
 * @Author: AN Hao, YUAN Hanqing
 * @Date: 2025-04-03 14:21:13
 * @LastEditors: YUAN Hanqing
 * @LastEditTime: 2025-04-11 00:21:39
 * @FilePath: /motioncontrol_a1pro/src/uplimb_interface/include/uplimb_interface/pid_msd.h
 */
#ifndef UL_SRC_UL_CONTROLLER_PID_MSD_H_
#define UL_SRC_UL_CONTROLLER_PID_MSD_H_

#include <memory>
#include <iostream>
#include <vector>
#include <deque>
#include <cmath>
#include <cassert>
#include <optional>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "ul/std/common.h"

namespace ul {
namespace controller {
class RealTimeGaussianFilter {
private:
    int window_size;
    int window_size_max = 101;
    double sigma;
    int dimension;
    bool initialized;
    std::vector<std::deque<double>> data_buffers; // 每维一个缓存
    std::vector<double> kernel;
    std::vector<double> causalKernel;

    std::vector<double> createKernel(int size, double sigma);

public:
    RealTimeGaussianFilter(int dimension, int window_size = 21, double sigma = 3.0);
    ~RealTimeGaussianFilter();

    std::vector<double> update(const std::vector<double>& new_values);

    // 重设滤波参数
    bool resetParas(double sigma, int window_size);
};
/////////////////////////////////////////////////
class SG_Predictor {
public:
    /**
     * @param windowTime   滑动窗口时长（秒）
     * @param polyOrder    多项式阶数
     * @param dim          每个观测向量的维度
     */
    SG_Predictor(int dim, double windowTime, int polyOrder)
        : windowTime(windowTime), polyOrder(polyOrder), dim(dim)
    {
        assert(windowTime > 0);
        assert(dim > 0);
    }

    /** 添加新的 (y, Δt) 向量点 **/
    void addData(const Eigen::VectorXd& y, double delta_time) {
        assert((int)y.size() == dim);
        double t_new = time.empty() ? 0.0 : time.back() + delta_time;
        data.push_back(y);
        time.push_back(t_new);
        // 淘汰过期点
        while (!time.empty() && t_new - time.front() > windowTime) {
            data.pop_front();
            time.pop_front();
        }
    }

    /** 判断是否已有足够点进行拟合 **/
    bool ready() const {
        return data.size() >= size_t(polyOrder + 1);
    }

    /** 重置所有缓存数据 **/
    void reset() {
        data.clear();
        time.clear();
    }

    /** 拟合并返回系数矩阵 A：(polyOrder+1)×dim **/
    Eigen::MatrixXd fitCoeffs() const {
        int N = polyOrder + 1;
        int n = data.size();
        // 数据点不足时退化为常数
        if (n < N) {
            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N, dim);
            if (n > 0) {
                A.row(0) = data.back().transpose();
            }
            return A;
        }
        // 计算每个 tau
        std::vector<double> taus(n);
        double t0 = time.back();
        for (int i = 0; i < n; ++i) taus[i] = time[i] - t0;

        // 重建 M_loc, d_loc
        Eigen::MatrixXd M_loc = Eigen::MatrixXd::Zero(N, N);
        Eigen::MatrixXd d_loc = Eigen::MatrixXd::Zero(N, dim);
        std::vector<double> pows;
        for (int i = 0; i < n; ++i) {
            // 计算 powers
            pows.assign(N, 1.0);
            for (int k = 1; k < N; ++k) pows[k] = pows[k-1] * taus[i];
            // 累加 M_loc 和 d_loc
            for (int k = 0; k < N; ++k) {
                d_loc.row(k) += pows[k] * data[i].transpose();
                for (int j = 0; j < N; ++j) {
                    M_loc(k, j) += pows[k] * pows[j];
                }
            }
        }
        // 求解最小二乘
        return M_loc.llt().solve(d_loc);
    }

    /** 预测未来 t = t0 + futureT 时刻的向量位置 **/
    Eigen::VectorXd predictPosition(double futureT) const {
        auto A = fitCoeffs();  // (N×dim)
        int N = polyOrder + 1;
        Eigen::VectorXd tpow(N);
        tpow(0) = 1.0;
        for (int i = 1; i < N; ++i) tpow(i) = tpow(i-1) * futureT;
        // v = A^T * tpow
        return A.transpose() * tpow;
    }

    /** 预测速度：一阶导数 **/
    Eigen::VectorXd predictVelocity(double futureT) const {
        auto A = fitCoeffs();
        Eigen::VectorXd v = Eigen::VectorXd::Zero(dim);
        for (int i = 1; i <= polyOrder; ++i) {
            double coef = double(i) * pow(futureT, i-1);
            v += coef * A.row(i).transpose();
        }
        return v;
    }

    /** 预测加速度：二阶导数 **/
    Eigen::VectorXd predictAcceleration(double futureT) const {
        auto A = fitCoeffs();
        Eigen::VectorXd v = Eigen::VectorXd::Zero(dim);
        for (int i = 2; i <= polyOrder; ++i) {
            double coef = double(i*(i-1)) * pow(futureT, i-2);
            v += coef * A.row(i).transpose();
        }
        return v;
    }

private:
    double windowTime;
    int polyOrder;
    int dim;
    std::deque<Eigen::VectorXd> data;
    std::deque<double>   time;
};
//////////////////////////////////////////
class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter(int dim_x, int dim_z)
        : x_(Eigen::VectorXd::Zero(dim_x)),
          P_(Eigen::MatrixXd::Identity(dim_x, dim_x)),
          F_(Eigen::MatrixXd::Identity(dim_x, dim_x)),
          H_(Eigen::MatrixXd::Zero(dim_z, dim_x)),
          Q_(Eigen::MatrixXd::Identity(dim_x, dim_x)),
          R_(Eigen::MatrixXd::Identity(dim_z, dim_z)),
          dim_x_(dim_x), dim_z_(dim_z) {}

    void init(const Eigen::VectorXd& x0,
              const Eigen::MatrixXd& P,
              const Eigen::MatrixXd& Q,
              const Eigen::MatrixXd& R,
              const Eigen::MatrixXd& F,
              const Eigen::MatrixXd& H) {
        x_ = x0;
        P_ = P;
        Q_ = Q;
        R_ = R;
        F_ = F;
        H_ = H;
    }

    void predict() {
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    void update(const Eigen::VectorXd& z) {
        Eigen::VectorXd y = z - H_ * x_;
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
        x_ = x_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(dim_x_, dim_x_) - K * H_) * P_;
    }

    const Eigen::VectorXd& state() const { return x_; }

    bool reset(const Eigen::VectorXd& initial_state) {
        if (initial_state.size() == dim_x_) {
            x_ = initial_state;
            P_ = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
            return true;
        } else {
            return false;
        }    
    }

private:
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    int dim_x_, dim_z_;
};


class MultiEKF {
public:
    MultiEKF(int N, double dt)
        : N_(N), dt_(dt) {
        // 状态转移矩阵（位置、速度、加速度）
        Eigen::MatrixXd F3(3, 3);
        F3 << 1, dt, 0.5 * dt * dt,
              0, 1, dt,
              0, 0, 1;

        // 观测矩阵：位置和速度
        Eigen::MatrixXd H3(2, 3);
        H3 << 1, 0, 0,
              0, 1, 0;

        // 过程噪声
        Eigen::MatrixXd Q3(3, 3);
        Q3 << 0.001, 0, 0,
              0, 0.01, 0,
              0, 0, 0.01;

        // 观测噪声：位置和位置差分（速度）的噪声
        Eigen::MatrixXd R2(2, 2);
        R2 << 0.1, 0,
              0,  0.01;

        // 初始状态与协方差
        Eigen::VectorXd x03 = Eigen::VectorXd::Zero(3);
        Eigen::MatrixXd P3 = Eigen::MatrixXd::Identity(3, 3);

        filters_.reserve(N_);
        for (int i = 0; i < N_; ++i) {
            ExtendedKalmanFilter ekf(3, 2);  // 注意观测维度是2
            ekf.init(x03, P3, Q3, R2, F3, H3);
            filters_.push_back(ekf);
        }
    }

    void predictAll() {
        for (auto& ekf : filters_) {
            ekf.predict();
        }
    }

    void updateAll(const std::vector<std::optional<std::pair<double, double>>>& measurements) {
        for (int i = 0; i < N_; ++i) {
            if (measurements[i].has_value()) {
                Eigen::VectorXd z(2);
                z << measurements[i]->first,  // 位置
                     measurements[i]->second; // 差分估计的速度
                filters_[i].update(z);
            }
        }
    }

    std::vector<Eigen::VectorXd> getStates() const {
        std::vector<Eigen::VectorXd> states;
        states.reserve(N_);
        for (const auto& ekf : filters_) {
            states.push_back(ekf.state().cast<double>());
        }
        return states;
    }

    bool resetAll(const std::vector<Eigen::VectorXd>& initial_state) {
        bool success = true;
        for (int i = 0; i < N_; ++i) {
            success &= filters_[i].reset(initial_state[i]);
        }
        return success;
    }

private:
    int N_;
    double dt_;
    std::vector<ExtendedKalmanFilter> filters_;
};
//////////////////////////////////////////////////////////////////////////////////
typedef struct
{
    std::vector<double> x_target;
    double time;
} PID_MSD_TARGET_STRUCT;

class PID_MSD
{
private:
    int dimension;
    int window_size;
    double sigma;

    PID_MSD_TARGET_STRUCT next_target, timeout_target;

    // PID参数
    double Kp;  // 比例增益
    double Ki;  // 积分增益
    double Kd;  // 微分增益

    // 弹簧阻尼质量块系统参数
    double m; // 质量 (kg)
    double k; // 弹簧系数 (N/m)
    double c; // 阻尼系数 (Ns/m)

    double dt;

    std::unique_ptr<RealTimeGaussianFilter> filter;
    std::unique_ptr<MultiEKF> ekf_predictor;
    std::unique_ptr<SG_Predictor> sg_predictor;


public:
    bool initialized;
    // 系统状态
    std::vector<double> x_t;        // 目标位置
    std::vector<double> x_t_f;      // 目标位置滤波
    std::vector<double> x;          // 位置
    std::vector<double> v;          // 速度
    std::vector<double> a;          // 加速度
    std::vector<double> integral;   // 误差积分
    std::vector<double> prev_error; // 上一步误差

    double current_time;
    bool blocked;
    bool used;
    bool timeout;
    double delay_time = 0.0;
    double update_time = 0.0;


    std::vector<double> x_target;
    std::vector<double> x_init;
    double time;
    
    
    PID_MSD(int dimension, double Kp, double Ki, double Kd, double m, double c, double k, double dt, int window_size, double sigma);
    ~PID_MSD();

    // 加载目标和初值
    bool load_target(std::vector<double> x_target, double time, std::vector<double> x_init, std::vector<double> v_init);
    // 更新系统状态
    bool update();
    // 重置系统
    bool resetSys();
    // 重设阻尼和滤波窗口长度参数
    bool resetParas(double c, int window_size);
};
}  // namespace controller
}  // namespace ul

#endif // UL_SRC_UL_CONTROLLER_PID_MSD_H_