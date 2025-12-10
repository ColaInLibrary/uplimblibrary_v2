/**
 ******************************************************************************
 * @Description   : 单边S型速度轨迹规划
 * @author        : AN Hao
 * @Date          : 25-2-10
 * @Version       : 0.0.1
 * @File          : SingleSShapeVelocity.h
 ******************************************************************************
 */

#ifndef UL_SRC_UL_MATH_SINGLESSHAPEVELOCITY_H_
#define UL_SRC_UL_MATH_SINGLESSHAPEVELOCITY_H_
#include <ul/std/common.h>
#include <ul/math/algorithm.h>

#include <cmath>
#include <iostream>

#include "Real.h"

namespace ul {
namespace math {
template <typename T>
class SingleSShapeVelocity {
 public:

  bool interpolate() {
    case_type_ = 0;
    // 不用处理的情况
    if (abs(ve_ - v0_) < 1e-5) { return false; }  // 前后速度差为0， 无需规划
    // std::cout << "a0_ = " << a0_ << std::endl;

    // 调整正负向，包括初始速度、终止速度、初始加速度
    T err = ve_ - v0_;
    delta_ = ::ul::math::sign(err);
    T jmin_ = -abs(jm_);  // min jerk
    T jmax_ = abs(jm_);   // max jerk
    T amin_ = -abs(am_);
    T amax_ = abs(am_);

    T amax = (delta_ + 1) / 2 * amax_ + (delta_ - 1) / 2 * amin_;
    T jmax = (delta_ + 1) / 2 * jmax_ + (delta_ - 1) / 2 * jmin_;
    T amin = (delta_ + 1) / 2 * amin_ + (delta_ - 1) / 2 * amax_;
    T jmin = (delta_ + 1) / 2 * jmin_ + (delta_ - 1) / 2 * jmax_;

    // 修正后，一定保证 初速度 < 终止速度
    T v0 = delta_ * v0_;  // 初始速度
    T ve = delta_ * ve_;  // 终止速度
    T a0 = (delta_ + 1) / 2 * a0_ + (delta_ - 1) / 2 * a0_;  // 初始加速度
    ae_ = 0.0;

    // compute time
    if (a0 < 0) {  // 初始加速度小于0，需要先减速再加速
      case_type_ = 1;
      if (j0_ == 0) {
        ::std::cout << "Denominator is zero." << ::std::endl;
        return false;
      }
      Tj0_ = abs(a0 / j0_);
      v0_prime_ = v0 + a0 * Tj0_ + j0_ * Tj0_ * Tj0_ / 2;
      a0_prime_ = 0.0;
    } else {
      Tj0_ = 0.0;
      v0_prime_ = v0;
      a0_prime_ = a0;
    }

    Tj1_ = abs(amax - a0_prime_)/jmax;
    Tj2_ = amax / jmax;
    T alima;
    delta_v_ = ve - v0_prime_;                                      // 需要完成的速度变化量
    delta_v_m1_ = (a0_prime_ + amax) * Tj1_ / 2 + amax * Tj2_ / 2;  // 没有匀加速段时能走过的最大速度变化量
    delta_v_m2_ = a0_prime_ * a0_prime_ / (2 * jmax);               // 按照当前jerk值，最小的速度变化量
    T Tj2_max = 2 * delta_v_ / a0_prime_;                           // 如果只有减速段，计算出的Tj2时间, 实际上所需的时间一定比该值小，比该值大会导致速度过冲
    if (delta_v_m1_ < delta_v_) {                                   // 有匀加速段
      case_type_ = 2;
      alima = amax;  
      Ta_ = (2*delta_v_ - (a0_prime_ + alima)*Tj1_ - alima*Tj2_) / (2*alima);
    } else {                                                        // 没有 匀加速段
      Ta_ = 0;
      if (delta_v_m2_ >= delta_v_) {                                 // 按照最大jerk运动，只能更大的jerk，这种情况下最终加速度不能为0
        case_type_ = 4;
        // jmax =  jm_;
        // Tj1_ = 0;   
        // j1_ = 0;
        // Tj2_ = Tj2_max;                                               // 这里其实Tj2_只是给了一个上限，没有什么实际用途
        // jmax = a0_prime_ / Tj2_;
        // alima = a0_prime_;
        Tj2_ = 0;
        j2_ = 0;
        if (a0_prime_*a0_prime_ - 2*jm_*delta_v_ < 0) throw std::runtime_error("sqrt negative error");
        alima = sqrt(a0_prime_*a0_prime_ - 2*jm_*delta_v_);
        j1_ = -jm_;
        Tj1_ = abs((a0_prime_ - alima) / j1_);
      } else {
        case_type_ = 3;
        alima = sqrt((2*jmax*delta_v_+ a0_prime_*a0_prime_)/2);
        Tj1_ = abs(alima - a0_prime_) / jmax;                         // 能按照最大jerk，加速度先增大再减小
        j1_ = jmax;
        Tj2_ = alima / jmax;
        j2_ = -jmax;
      }
    }


    // 计算总时间
    T_ = Tj0_ + Ta_ + Tj1_ + Tj2_;
    // 计算最大加速度
    am_ = alima;
    return true;
  }


  void reset_paras() {
    Tj0_ = 0;
    Tj1_ = 0;
    Ta_ = 0; 
    Tj2_ = 0;
    T_ = 0;
    replan_flag = false;
  }

  /**
   * @brief : Re-calculate the planning parameters
   * @param r : ratio
   * @return : Current Jerk
   */
  void replan(const Real& r) {
    // std::cout << "--------Before Replan:" << std::endl;
    // std::cout << "case_type_ = " << case_type_ << std::endl;
    // std::cout << "am_ = " << am_ << ", jm_ = " << jm_ << std::endl;
    // std::cout << "Tj1_ = " << Tj1_ << ", Ta_ = " << Ta_ << ", Tj2_ = " << Tj2_ << ", T_ = " << T_ << std::endl;
    T new_time = T_ / r;
    T new_ratio = (T_ - Tj0_) / (new_time - Tj0_);
    
    // 修正后，一定保证 初速度 < 终止速度
    T v0 = delta_ * v0_;  // 初始速度
    T ve = delta_ * ve_;  // 终止速度
    T a0 = (delta_ + 1) / 2 * a0_ + (delta_ - 1) / 2 * a0_;  // 初始加速度

    // Tj0_ 不进行变换
    Tj1_ = Tj1_ / new_ratio;
    Tj2_ = Tj2_ / new_ratio;
    Ta_ = Ta_ / new_ratio;
    T_ = Tj0_ + Tj1_ + Ta_ + Tj2_; 
    Real temp0 = (T_ - a0_prime_/jm_);
    Real temp1 = temp0 * temp0;
    Real temp2 = a0_prime_*a0_prime_/jm_ - 2*delta_v_;

    switch (case_type_) {
      case 1: // a0 < 0 的情况
        am_ = (2*(ve - v0_prime_) - a0_prime_ * Tj1_) / (Tj1_ + Tj2_ + 2*Ta_);
        if (Tj1_ < 1e-4) {
          j1_ = 0;
        } else {
          j1_ = (am_- a0_prime_)/Tj1_;  // j1_>0表示加加速，j1_<0表示减加速
        }
        j2_ = -jm_;
        ae_ = 0;
        break;
      
      case 2:
        am_ = (2*(ve - v0_prime_) - a0_prime_ * Tj1_) / (Tj1_ + Tj2_ + 2*Ta_);
        if (Tj1_ < 1e-4) {
          j1_ = 0;
        } else {
          j1_ = (am_- a0_prime_)/Tj1_;  // j1_>0表示加加速，j1_<0表示减加速
        }
        j2_ = -jm_;
        ae_ = 0;
        break;
      
      case 3:
        am_ = (2*(ve - v0_prime_) - a0_prime_ * Tj1_) / (Tj1_ + Tj2_);
        // std::cout << "case 3: am_ = " << am_ << std::endl;
        if (am_ < a0_prime_) {  // 重规划后最大加速度小于初值，变为减加速过程
          j1_ = -abs((am_ - a0_prime_) / Tj1_);
          j2_ = -abs(am_ / Tj2_);
        } else {
          j1_ = abs((am_ - a0_prime_) / Tj1_);
          j2_ = -abs(am_ / Tj2_);
        }
        ae_ = 0;
        break;
      
      case 4:
        if (abs(temp0) > 1e-5) {  // 说明不能按照最大jerk仅做减加速过程
          // j1_ = -jm_;
          // if ((-temp1/temp2 + 1/jm_) == 0) throw std::runtime_error("case4 wrong!");
          // j2_ = -1/(-temp1/temp2 + 1/jm_);
          // am_ = temp0 / (1/j1_ + 1/j2_);
          // // 这里需要重新计算Tj1_和Tj2_，其他情况则不需要
          // Tj1_ = abs((am_ - a0_prime_) / j1_);
          // Tj2_ = abs(am_ / j2_);
          Tj2_ = 0;
          j2_ = 0;
          am_ = 2*delta_v_/Tj1_ - a0_prime_;
          j1_ = (am_ - a0_prime_) / Tj1_;
          ae_ = delta_ * am_;
        }
        break;
      
      default:
        break;
    }

    // std::cout << "--------After Replan:" << std::endl;
    // std::cout << "r = " << r << ", new_time = " << new_time << ", new_ratio = " << new_ratio << std::endl;
    // std::cout << "ve = " << ve << ", v0_prime_ = " << v0_prime_ << ", a0_prime_ = " << a0_prime_ << std::endl;
    // std::cout << "Tj0_ = " << Tj0_ << ", Tj1_ = " << Tj1_ << ", Ta_ = " << Ta_ << ", Tj2_ = " << Tj2_ << ", T_ = " << T_ << std::endl;
    // std::cout << "am_ = " << am_ << ", j1_ = " << j1_ << ", j2_ = " << j2_ << ", jm_ = " << jm_ << std::endl;
    replan_flag = true;
  }

  /**
   * @brief : Calculate the acceleration
   * @param t : Current time
   * @return : Current Acceleration
   */
  T a(const Real& t) {
    T a0 = (delta_ + 1) / 2 * a0_ + (delta_ - 1) / 2 * a0_;  // 初始加速度
    if (t < 0) {
      return 0;
    } else if (t < Tj0_) {
      // std::cout << "t = " << t << ", a0 = " << a0 << ", j0_ = " << j0_ << std::endl;
      return delta_ * (a0 + j0_ * t);
    } else if (t < Tj0_ + Tj1_) {                // case 1: 加速度上升
      // std::cout << "t = " << t << std::endl;
      return delta_ * (a0_prime_ + j1_ * (t - Tj0_));
    } else if (t < Tj0_ + Tj1_+ Ta_) {          // case 2: 加速度匀速
      return delta_ * am_;
    } else if (t < T_) {                 // case 3: 加速度减小
      return delta_ * (am_ + j2_ * (t - Tj0_ - Ta_ - Tj1_));
    } else {                              // case 4: 匀速段
      return ae_;
    }
  }

  /**
   * @brief : Calculate the jerk
   * @param t : Current time
   * @return : Current Jerk
   */
  T j(const Real& t) {
    if (t < 0) {
      return 0;
    } else if (t < Tj0_) {                // case 1: 加速度上升
      return j0_ * delta_;
    } else if (t < Tj0_ + Tj1_) {                // case 1: 加速度上升
      return j1_ * delta_;
    } else if (t < T_ - Tj2_) {          // case 2: 加速度匀速
      return 0;
    } else if (t < T_) {                 // case 3: 加速度减小
      return j2_ * delta_;
    } else {                              // case 4: 加速度为0
      return 0;
    }
  }

  /**
   * @brief : Calculate the total time
   * @return
   */
  T t() const {
    return T_;
  }

  /**
   * @brief : Calculate the velocity
   * @param t : Current time
   * @return : Current Velocity
   */
  T v(const Real& t) const {  // time 赋值处理
    T v0 = delta_ * v0_;
    T ve = delta_ * ve_;
    T a0 = delta_ * a0_;
    if (t < 0) {
      return delta_ * v0;
    } else if (t < Tj0_) {         // case 1: 加速度上升至零
      return delta_ * (v0 + a0 * t + j0_ * pow(t, 2) / 2);
    } else if (t < Tj0_ + Tj1_) {
      return delta_ * (v0_prime_ + a0_prime_*(t - Tj0_) + j1_ * pow(t-Tj0_, 2) / 2);
    } else if (t < T_ - Tj2_) {   // case 2: 加速度匀速
      return delta_ * (v0_prime_ + a0_prime_*Tj1_ + j1_ * pow(Tj1_, 2) / 2 + am_*(t - Tj0_ - Tj1_));
    } else if (t < T_) {          // case 3: 加速度减小
      return delta_ * (v0_prime_ + a0_prime_*Tj1_ + j1_ * pow(Tj1_, 2) / 2 + am_*Ta_ + am_*(t - Tj0_ - Ta_ - Tj1_) + j2_ * pow(t - Tj0_ - Ta_ - Tj1_, 2) / 2);
    } else {                       // case 4: 加速度为0
      return delta_ * ve;
    }
  }

  T a0_;    // Initial acceleration
  T am_;    // Max acceleration
  T ae_;    // End acceleration
  T jm_;
  T v0_;    // Initial velocity
  T ve_;    // End velocity
  T j0_;  // 最大jerk值，常数，不会再重规划时刷新

 protected:
 private:
  Real Tj0_, Tj1_, Ta_, Tj2_, j1_, j2_;
  Real T_;
  T delta_;  // 初始速度和终止速度的方向标志
  Real delta_v_, delta_v_m1_, delta_v_m2_;
  Real v0_prime_, a0_prime_;
  unsigned int case_type_;
  bool replan_flag = false;
};
}  // namespace math
}  // namespace ul
#endif  // UL_SRC_UL_MATH_SINGLESSHAPEVELOCITY_H_