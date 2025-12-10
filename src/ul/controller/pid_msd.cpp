/*
 * @Description:  
 * @Author: AN Hao, YUAN Hanqing
 * @Date: 2025-06-09 13:59:15
 * @LastEditors: YUAN Hanqing
 * @LastEditTime: 2025-07-03 20:43:20
 * @FilePath: /uplimlibrary/src/ul/controller/pid_msd.cpp
 */
/*
 * @Description:  
 * @Author: AN Hao, YUAN Hanqing
 * @Date: 2025-04-03 14:05:30
 * @LastEditors: YUAN Hanqing
 * @LastEditTime: 2025-04-11 09:13:20
 * @FilePath: /motioncontrol_a1pro/src/uplimb_interface/src/pid_msd.cpp
 */
#include "pid_msd.h"

namespace ul {
namespace controller {
RealTimeGaussianFilter::RealTimeGaussianFilter(int dimension, int window_size, double sigma)
    : dimension(dimension), sigma(sigma), initialized(false)
{
    if (window_size % 2 == 0) {
        window_size += 1; 
    }
    this->window_size = window_size;
    kernel = createKernel(window_size, sigma);
    data_buffers.resize(dimension);  // 每维初始化一个队列
}

RealTimeGaussianFilter::~RealTimeGaussianFilter() {}

std::vector<double> RealTimeGaussianFilter::update(const std::vector<double>& new_values) {
    std::vector<double> smoothed_values(dimension, 0.0);
    if (new_values.size() != dimension) {
        std::cerr << "Dimension mismatch in update!" << std::endl;
        return new_values;
    }

    // 初始化：每个维度都填满
    if (!initialized) {
        for (int d = 0; d < dimension; ++d) {
            for (int i = 0; i < window_size_max; ++i) {
                // data_buffers[d].push_back(new_values[d]);
                data_buffers[d].push_back(0.0);
            }
        }
        initialized = true;
    } else {
        for (int d = 0; d < dimension; ++d) {
            data_buffers[d].push_back(new_values[d]);
            // 控制缓存长度
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

std::vector<double> RealTimeGaussianFilter::createKernel(int size, double sigma) {
    std::vector<double> kernel_full(size, 0.0);
    double sum = 0.0;
    double center = (size - 1) / 2.0;
    for (int i = 0; i < size; ++i) {
        kernel_full[i] = std::exp(-0.5 * std::pow((i - center) / sigma, 2));
        sum += kernel_full[i];
    }
    for (int i = 0; i < size; ++i) {
        kernel_full[i] /= sum;
    }

    int start = size / 2;
    std::vector<double> causal(kernel_full.begin() + start, kernel_full.end());
    double causalSum = 0.0;
    for (double val : causal) {
        causalSum += val;
    }
    for (double &val : causal) {
        val /= causalSum;
    }
    causalKernel = causal;
    return kernel_full;
}

bool RealTimeGaussianFilter::resetParas(double sigma, int window_size) {
    if (window_size % 2 == 0) {
        window_size += 1;
    }

    if (window_size > 2 * window_size_max - 1 || window_size < 31) {
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
/////////////////////////////////////////////////////////
PID_MSD::PID_MSD(int dimension = 1, double Kp = 300.0, double Ki = 80.0, double Kd = 15.0, double m = 1.0, double c = 15.0, double k = 0.0, double dt = 0.001, int window_size = 201, double sigma = 15)
    : initialized(false), dimension(dimension), Kp(Kp), Ki(Ki), Kd(Kd), m(m), c(c), k(k), dt(dt), window_size(window_size), sigma(sigma), blocked(false), used(false), timeout(false) {
    filter = std::make_unique<RealTimeGaussianFilter>(dimension, window_size, sigma);
    ekf_predictor = std::make_unique<MultiEKF>(dimension, dt);
    sg_predictor = std::make_unique<SG_Predictor>(dimension, 0.2, 3);
    x_target.resize(dimension, 0.0);
    x_init.resize(dimension, 0.0);
    x_t.resize(dimension, 0.0);  
    x_t_f.resize(dimension, 0.0);  
    x.resize(dimension, 0.0);          
    v.resize(dimension, 0.0);          
    a.resize(dimension, 0.0);          
    integral.resize(dimension, 0.0);   
    prev_error.resize(dimension, 0.0); 
}

PID_MSD::~PID_MSD() {}

bool PID_MSD::load_target(std::vector<double> x_target, double time, std::vector<double> x_init, std::vector<double> v_init) {
    this->next_target.time = time;
    this->next_target.x_target = x_target;

    if (!initialized) { // 第一次
        this->x = x_init;
        this->v = v_init;

        this->x_init = x_init;
        this->time = time;
        this->x_target = x_target;
        this->current_time = 0.0;
        
        initialized = true;
        used = true;

        std::vector<Eigen::VectorXd> initial_state(this->dimension, Eigen::VectorXd::Zero(3));
        for (int i = 0; i < this->dimension; i++) {
            initial_state[i][0] = x_init[i];
            initial_state[i][1] = v_init[i];
        } 
        ekf_predictor->resetAll(initial_state);
        sg_predictor->reset();
    } else {
        used = false;
    }
    return true;
}

bool PID_MSD::update() {
    if (!initialized) {
        std::cout << "PID_MSD: initialization is not yet complete!" << std::endl;
        return false;
    }
////////////////////////////////////////////////////////
    // std::vector<std::optional<std::pair<double, double>>> meas_t;
    // ekf_predictor->predictAll();
    // Eigen::VectorXd x_t_vec;
    // if (delay_time == 0.0) {
    //     for(int i=0; i<dimension; i++) {
    //         x_t[i] = x_init[i] + (current_time + dt) * ((x_target[i] - x_init[i]) / time);
    //     }
    //     x_t_vec = Eigen::Map<Eigen::VectorXd>(x_t.data(), x_t.size());
    //     sg_predictor->addData(x_t_vec, update_time + dt); // 更新滤波输入数据
    //     update_time = 0.0;
    // } else {
    //     update_time += dt;
    // }
    // for(int i=0; i<dimension; i++) {
    //     x_t_f[i] = sg_predictor->predictPosition(update_time)[i];
    //     // meas_t.push_back(std::make_pair(sg_predictor->predictPosition(update_time)[i], sg_predictor->predictVelocity(update_time)[i])); // 获取预测数据
    // }
    // ekf_predictor->updateAll(meas_t);
    // auto states = ekf_predictor->getStates();
    // for(int i=0; i<dimension; i++) {
    //     x_t_f[i] = states[i][0];
    // } 
///////////////////////////////////////////////////////////
    if (current_time >= time) {
        if (!used) {
            if (delay_time == 0.0) {  // 及时
                // std::cout << "***********及时*********** delay_time: " << delay_time << " current_time: " << current_time << std::endl;
                x_init = x_target;
                x_target = next_target.x_target;
            } else {  // 超时
                if (delay_time > 2*dt) { //超时超过2dt
                    // std::cout << RED << "***********超时超过2dt*********** delay_time: " << delay_time << " current_time: " << current_time << RESET << std::endl;
                    x_init = x_t;
                    x_target = next_target.x_target;
                } else {  //超时2dt以内
                    // std::cout << YELLOW << "***********超时2dt以内*********** delay_time: " << delay_time << " current_time: " << current_time << " time: " << time <<  RESET << std::endl;
                    x_init = x_t;
                    for(int i=0; i<dimension; i++) {
                        x_target[i] = next_target.x_target[i] + (x_t[i] - x_target[i]);
                    }
                }
            }
            current_time = 0.0; 
            time = next_target.time;
            delay_time = 0.0;
            blocked = true;
            used = true;
        } else { //未到
            // std::cout << "***********未到*********** delay_time: " << delay_time << " current_time: " << current_time  << std::endl;
            delay_time += dt;
            blocked = false;
        }
    }
///////////////////////////////////////////////////////////////////
// if (current_time >= time) {
//     Eigen::VectorXd x_t_vec;
//     if (!used) {
//         if (delay_time == 0.0) {  // 当前帧及时送达
//             if (!timeout) { // 上一帧及时送达
//                 x_init = x_target;
//                 x_target = next_target.x_target;
//             } else { // 上一帧超时送达
//                 std::cout << YELLOW << "***********上一帧超时送达***********" <<  RESET << std::endl;
//                 timeout_target = next_target; // 记录超时送达帧
//                 x_t_vec = Eigen::Map<Eigen::VectorXd>(next_target.x_target.data(), next_target.x_target.size()) - Eigen::Map<Eigen::VectorXd>( timeout_target.x_target.data(),  timeout_target.x_target.size());
//                 std::cout << "err_o: " << x_t_vec.segment<7>(7).transpose() << std::endl;
//                 x_init = x_target;
//                 for(int i=0; i<dimension; i++) {
//                     x_target[i] = next_target.x_target[i] + (x_target[i] - timeout_target.x_target[i]);
//                 }
//                 x_t_vec = Eigen::Map<Eigen::VectorXd>(x_target.data(), x_target.size()) - Eigen::Map<Eigen::VectorXd>(x_init.data(), x_init.size());
//                 std::cout << "err_m: " << x_t_vec.segment<7>(7).transpose() << std::endl;
//             }
//         } else {  // 超时送达
//             if (delay_time < time) {  // 当前帧超时送达
//                 timeout = true;
//                 timeout_target = next_target; // 记录超时送达帧
//                 std::cout << YELLOW << "***********超时time以内*********** delay_time: " << delay_time <<  RESET << std::endl;
//                 x_t_vec = Eigen::Map<Eigen::VectorXd>(next_target.x_target.data(), next_target.x_target.size()) - Eigen::Map<Eigen::VectorXd>(x_target.data(), x_target.size());
//                 std::cout << "err_o: " << x_t_vec.segment<7>(7).transpose() << std::endl;
//                 for(int i=0; i<dimension; i++) {
//                     x_target[i] = next_target.x_target[i] + (x_t[i] - x_target[i]);
//                 }
//                 x_init = x_t;
//                 x_t_vec = Eigen::Map<Eigen::VectorXd>(x_target.data(), x_target.size()) - Eigen::Map<Eigen::VectorXd>(x_init.data(), x_init.size());
//                 std::cout << "err_m: " << x_t_vec.segment<7>(7).transpose() << std::endl;
//             } else { // 无关帧送达
//                 timeout = false;
//                 x_init = x_target;
//                 x_target = next_target.x_target;
//             }
//         }
//         current_time = 0.0; 
//         time = next_target.time;
//         delay_time = 0.0;
//         blocked = true;
//         used = true;
//     } else { //未到
//         delay_time += dt;
//         blocked = false;
//     }
// }    
///////////////////////////////////////////////////////////////////
    // Eigen::VectorXd x_t_vec;
    // if (delay_time == 0.0) {
    //     for(int i=0; i<dimension; i++) {
    //         x_t[i] = x_init[i] + (current_time + dt) * ((x_target[i] - x_init[i]) / time);
    //     }
    //     x_t_vec = Eigen::Map<Eigen::VectorXd>(x_t.data(), x_t.size());
    //     sg_predictor->addData(x_t_vec, update_time + dt); // 更新滤波输入数据
    //     update_time = 0.0;
    // } else {
    //     update_time += dt;
    // }
    // for(int i=0; i<dimension; i++) {
    //     x_t_f[i] = sg_predictor->predictPosition(update_time)[i];
    // } 
///////////////////////////////////////////////////////////////////    
    // std::vector<std::optional<std::pair<double, double>>> meas_t;
    // Eigen::VectorXd x_t_vec;
    // ekf_predictor->predictAll();
    // if (delay_time == 0.0) {
    //     for(int i=0; i<dimension; i++) {
    //         x_t[i] = x_init[i] + (current_time + dt) * ((x_target[i] - x_init[i]) / time);
    //     }
    //     x_t_vec = Eigen::Map<Eigen::VectorXd>(x_t.data(), x_t.size());
    //     sg_predictor->addData(x_t_vec, dt); // 更新滤波输入数据
    //     for(int i=0; i<dimension; i++) {
    //         meas_t.push_back(std::make_pair(sg_predictor->predictPosition(0.0)[i], sg_predictor->predictVelocity(0.0)[i])); // 获取预测数据
    //     }
    // } else {
    //     if (delay_time < 3*dt) {  //下一目标信息最多允许延迟2dt 
    //         for(int i=0; i<dimension; i++) {
    //             meas_t.push_back(std::make_pair(sg_predictor->predictPosition(delay_time)[i], sg_predictor->predictVelocity(delay_time)[i]));  // 获取预测数据
    //         }
    //     } else {  // 超过2dt没有接收到新数据
    //         x_t_vec = Eigen::Map<Eigen::VectorXd>(x_t.data(), x_t.size());
    //         if (delay_time < 4*dt) { // 3dt
    //             sg_predictor->addData(x_t_vec, delay_time + dt);  // 更新滤波输入数据
    //         } else { // >3dt
    //             sg_predictor->addData(x_t_vec, dt);  // 更新滤波输入数据
    //         }
    //         for(int i=0; i<dimension; i++) {
    //             meas_t.push_back(std::make_pair(sg_predictor->predictPosition(0.0)[i], sg_predictor->predictVelocity(0.0)[i])); // 获取预测数据
    //         }
    //     }
    // }

    // ekf_predictor->updateAll(meas_t);
    // auto states = ekf_predictor->getStates();
    // for(int i=0; i<dimension; i++) {
    //     x_t_f[i] = states[i][0];
    // } 
////////////////////////////////////////////////////////////////////
    if (delay_time < 5*dt) {
        for(int i=0; i<dimension; i++) {
            x_t[i] = x_init[i] + (current_time + dt) * ((x_target[i] - x_init[i]) / time);
        }
    }
    x_t_f = x_t;
/////////////////////////////////////////////////////////////////////
    std::vector<double> error(dimension);
    std::vector<double> derivative(dimension);
    std::vector<double> F(dimension);
    std::vector<double> acceleration(dimension);

    for(int i=0; i < dimension; i++) {
        error[i] = x_t_f[i] - x[i];  // 当前误差
        // PID 计算
        integral[i] += error[i] * dt;
        derivative[i] = (error[i] - prev_error[i]) / dt;
        F[i] = Kp * error[i] + Ki * integral[i] + Kd * derivative[i];

        // 系统动力学方程
        acceleration[i] = (F[i] - c * v[i] - k * x[i]) / m;
    }
    
    // 滤波
    a = filter->update(acceleration);
    // 欧拉积分
    for(int i=0; i < dimension; i++) {    
        v[i] += a[i] * dt;
        x[i] += v[i] * dt;
    }

    prev_error = error;
    current_time += dt; 

    bool reach = true;
    for(int i=0; i < dimension; i++) {
        reach &= (std::abs(x[i] - x_target[i]) < 0.005) & (std::abs(a[i]) < 0.01) & (std::abs(v[i]) < 0.001);
    }
      
    if (reach) {  // 到达目标位置返回true
        return true;
    } else {
        return false;
    }
}

bool PID_MSD::resetSys() {
    initialized = false;
    blocked = false;
    used = false;
    timeout = false;
    delay_time = 0.0;
    for(int i=0; i < dimension; i++) {
        integral[i] = 0.0;   
        prev_error[i] = 0.0;
    }
    filter = std::make_unique<RealTimeGaussianFilter>(dimension, window_size, sigma);
    ekf_predictor = std::make_unique<MultiEKF>(dimension, dt);
    sg_predictor = std::make_unique<SG_Predictor>(dimension, 0.2, 3);
    return true;
}

bool PID_MSD::resetParas(double c, int window_size) {
    this->c = c;
    if (filter->resetParas(this->sigma, window_size)) {
        this->window_size = window_size;
        return true;
    } else {
        return false;
    }
}
//////////////////////////////////////////////////////////
}  // namespace controller
}  // namespace ul