/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-1-14
 * @Version       : 0.0.1
 * @File          : SShapeVelocity.h
 ******************************************************************************
 */

#ifndef UL_SRC_UL_MATH_SSHAPEVELOCITY_H_
#define UL_SRC_UL_MATH_SSHAPEVELOCITY_H_
#include <ul/std/common.h>
#include <ul/math/algorithm.h>

#include <cmath>
#include <iostream>

#include "Real.h"

namespace ul {
namespace math {
template <typename T>
class SShapeVelocity {
 public:
  /**
   * @brief : Calculate the acceleration
   * @param t : Current time_
   * @return : Current Acceleration
   */
  T a(const Real& t) {
    if (t < 0) {
      return 0;
    } else if (t < Tj1_) {                // case 1: 加速度上升
//      ::std::cout << "case 1: t = " << t << ", Tj1_ = " << Tj1_ << ::std::endl;
      return delta_ * jm_ * t;
    } else if (t < Ta_ - Tj1_) {          // case 2: 加速度匀速
//      ::std::cout << "case 2: t = " << t << ", Ta_ = " << Ta_ << ::std::endl;
      return delta_ * am_;
    } else if (t < Ta_) {                 // case 3: 加速度减小
//      ::std::cout << "case 3: t = " << t << ", Ta_ = " << Ta_ << ::std::endl;
      return delta_ * (jm_ * (Ta_ - t));
    } else if (t < Ta_ + Tv_) {           // case 4: 加速度为0
//      ::std::cout << "case 4: t = " << t << ", Tv_ = " << Tv_ << ::std::endl;
      return 0;
    } else if (t < time_ - Td_ + Tj2_) {  // case 5: 减速度增大
//      ::std::cout << "case 5: t = " << t << ", Tj2_ = " << Tj2_ << ::std::endl;
      return delta_ * (-jm_ * (t - time_ + Td_));
    } else if (t < time_ - Tj2_) {        // case 6: 减速度匀速
//      ::std::cout << "case 6: t = " << t << ", Td_ = " << Td_ << ::std::endl;
      return delta_ * dm_;
    } else if (t < time_) {               // case 7: 减速度减小
//      ::std::cout << "case 7: t = " << t << ", time_ = " << time_ << ::std::endl;
      return delta_ * (-jm_ * (time_ - t));
    } else {                              // case 8: 超过规划时间的情况
//      ::std::cout << "case 8: t = " << t << ", time_ = " << time_ << ::std::endl;
      return 0;
    }
  }

  int interpolate() {
//    ::std::cout << BLUE << "x0_ = " << x0_ << ", xe_ = " << xe_ << RESET << ::std::endl;
    if (abs(am_) == 0) {
      ::std::cout << RED << "[ERROR SShape]: input parameter 'a' is zero!" << RESET << ::std::endl;
      return 0;
    }
    if (abs(vm_) == 0) {
      ::std::cout << RED << "[ERROR SShape]: input parameter 'v' is zero!" << RESET << ::std::endl;
      return 0;
    }
    
    T err = xe_ - x0_;
    delta_ = ::ul::math::sign(err);

    T jmin_ = -abs(jm_);  // min jerk
    T jmax_ = abs(jm_);   // max jerk
    T vmin_ = -abs(vm_);
    T vmax_ = abs(vm_);
    T amin_ = -abs(am_);
    T amax_ = abs(am_);

    T q0 = delta_ * x0_;
    T v0 = delta_ * v0_;
    T q1 = delta_ * xe_;
    T v1 = delta_ * ve_;

    T vmax = (delta_ + 1) / 2 * vmax_ + (delta_ - 1) / 2 * vmin_;
    T vmin = (delta_ + 1) / 2 * vmin_ + (delta_ - 1) / 2 * vmax_;
    T amax = (delta_ + 1) / 2 * amax_ + (delta_ - 1) / 2 * amin_;
    T amin = (delta_ + 1) / 2 * amin_ + (delta_ - 1) / 2 * amax_;
    T jmax = (delta_ + 1) / 2 * jmax_ + (delta_ - 1) / 2 * jmin_;
    T jmin = (delta_ + 1) / 2 * jmin_ + (delta_ - 1) / 2 * jmax_;

    if (abs(q0 - q1) < 1e-8) { return -1; }  // 轨迹长度为0， 无需规划

    // abs(v0) > vmax
    if (abs(v0) > abs(vmax)) {
      ::std::cout << RED << "[ERROR SShape]: v0 > vmax!" << RESET << ::std::endl;
      return 0;
    }
    if (abs(v1) > abs(vmax)) {
      ::std::cout << RED << " [Error SShape] vmax is less than v1!" << RESET << ::std::endl;
      return 0;
    }

    // compute time_
    T Tj_star = 0;
    T temp1 = sqrt(abs(v1 - v0) / jmax);
    T temp2 = amax / jmax;

    // 判断轨迹是否可以求解
    if (temp1 < temp2) {
      Tj_star = temp1;
     if ((q1 - q0) < Tj_star * (v0 + v1)) {
      ::std::cout << RED << "[ERROR SShape]: trajectory infeasible!" << RESET << ::std::endl;
      return 0;
     }
    } else {
      Tj_star = temp2;
     if ((q1 - q0) < 0.5 * (v0 + v1) * (Tj_star + abs(v1 - v0) / amax)) {
      ::std::cout << RED << "[ERROR SShape]: trajectory infeasible!" << RESET << ::std::endl;
      return 0;
     }
    }

    // assuming: v_lim = v_max
    if ((vmax - v0) * jmax < amax * amax) {
//     std::cout << "a_max is not reached!" << std::endl;
      Tj1_ = sqrt((vmax - v0) / jmax);
      Ta_ = 2 * Tj1_;
    } else {
//      std::cout << "a_max is reached!" << std::endl;
      Tj1_ = amax / jmax;
      Ta_ = Tj1_ + (vmax - v0) / amax;
    }
    // a_min = -a_max 
    if ((vmax - v1) * jmax < amax * amax) {
//      std::cout << "a_min is not reached!" << std::endl;
      Tj2_ = sqrt((vmax - v1) / jmax);
      Td_ = 2 * Tj2_;
    } else {
//      std::cout << "a_min is reached!" << std::endl;
      Tj2_ = amax / jmax;
      Td_ = Tj2_ + (vmax - v1) / amax;
    }

    Tv_ = (q1 - q0) / vmax - Ta_ / 2 * (1 + v0 / vmax) - Td_ / 2 * (1 + v1 / vmax);
    
    if (Tv_ > 1e-6) {       // 存在匀速运动的过程 case 1: v_lim = v_max
//      std::cout << "Robot can reach the desired max velocity!" << std::endl;
    } else {  // 不存在匀速运动的过程 Case2: v_lim < v_max
//      std::cout << "Desire max velocity is not reached!" << std::endl;
      Tv_ = 0;
      T delta = pow(amax, 4) / pow(jmax, 2) + 2 * (pow(v0, 2) + pow(v1, 2)) + amax * (4 * (q1 - q0) - 2 * amax / jmax * (v0 + v1));
      Tj1_ = amax / jmax;
      Tj2_ = amax / jmax;
      Ta_ = (pow(amax, 2) / jmax - 2 * v0 + sqrt(delta)) / (2 * amax);
      Td_ = (pow(amax, 2) / jmax - 2 * v1 + sqrt(delta)) / (2 * amax);
      
      T amax_r = amax;  // amax_r 表示用于迭代的最大加速度
      while(1) {
        if (Ta_ < 0 || Td_ < 0) {
          if ((Ta_ < 0) && (v0 > v1)) {
            // 这里只需要一个减速段即可
            Ta_ = 0;
            Tj1_ = 0;
            Td_ = 2 * (q1 - q0) / (v1 + v0);
            Tj2_ = (jmax * (q1 - q0) - sqrt(jmax * (jmax * pow((q1 - q0), 2) + pow(v0 + v1, 2) * (v1 - v0)))) / jmax / (v1 + v0);
          } else if ((Td_ < 0) && (v0 < v1)) {
            // 这里只需要一个加速段即可
            Td_ = 0;
            Tj2_ = 0;
            Ta_ = 2 * (q1 - q0) / (v1 + v0);
            Tj1_ = (jmax * (q1 - q0) - sqrt(jmax * (jmax * pow((q1 - q0), 2) - pow((v1 + v0), 2) * (v1 - v0)))) / jmax / (v1 + v0);
          } else {
            ::std::cout << RED << " [Error SShape] !{(Ta_ < 0) && (v0 > v1)} !{(Td_ < 0) && (v0 < v1)}" << RESET << ::std::endl;
            return 0;
          }
          break;
        } else {
          if (Ta_ < 2 * Tj1_ || Td_ < 2 * Tj2_) {
            amax_r = amax_r - amax * 0.01;
            if (amax_r < 0) {
              ::std::cout << RED << " [Error SShape] amax_r < 0 " << RESET << ::std::endl;
              return 0;
            }          
            delta = pow(amax_r, 4) / pow(jmax, 2) + 2 * (pow(v0, 2) + pow(v1, 2)) + amax_r * (4 * (q1 - q0) - 2 * amax_r / jmax * (v0 + v1));
            Tj1_ = amax_r / jmax;
            Tj2_ = amax_r / jmax;
            Ta_ = (pow(amax_r, 2) / jmax - 2 * v0 + sqrt(delta)) / (2 * amax_r);
            Td_ = (pow(amax_r, 2) / jmax - 2 * v1 + sqrt(delta)) / (2 * amax_r);
          } else {
            break;
          }
        }
      }
#ifdef DEBUG
      std::cout << "vlim = " << vlim << std::endl;
      std::cout << "alima = " << alima << std::endl;
      std::cout << "alimd = " << alimd << std::endl;
#endif
    }

    T alima = jmax * Tj1_;
    T alimd = -jmax * Tj2_;
    T vlim = v0 + (Ta_ - Tj1_) * alima;

    time_ = Ta_ + Td_ + Tv_;
//    ::std::cout << "time_ = " << time_ << ", Ta_ = " << Ta_ << ", Td_ = " << Td_ << ", Tv_ = " << Tv_ << std::endl;
    jm_ = jmax;
    dm_ = alimd;
    am_ = alima;
    vm_ = vlim;
    return 1;
  }

  /**
   * @brief : Calculate the jerk
   * @param t : Current time_
   * @return : Current Jerk
   */
  T j(const Real& t) {
    if (t < 0) {
      return 0;
    } else if (t < Tj1_) {                // case 1: 加速度上升
      return delta_ * jm_;
    } else if (t < Ta_ - Tj1_) {          // case 2: 加速度匀速
      return 0;
    } else if (t < Ta_) {                 // case 3: 加速度减小
      return delta_ * (-jm_);
    } else if (t < Ta_ + Tv_) {           // case 4: 加速度为0
      return 0;
    } else if (t < time_ - Td_ + Tj2_) {  // case 5: 减速度增大
      return delta_ * (-jm_);
    } else if (t < time_ - Tj2_) {        // case 6: 减速度匀速
      return 0;
    } else if (t < time_) {               // case 7: 减速度减小
      return delta_ * jm_;
    } else {                              // case 8: 超过规划时间的情况
      return 0;
    }
  }

  /**
   * @brief : Calculate the total time_
   * @return
   */
  T t() const {
    return Ta_ + Tv_ + Td_;
  }

  /**
   * @brief : Calculate the velocity
   * @param t : Current time_
   * @return : Current Velocity
   */
  T v(const Real& t) const {  // time_ 赋值处理
    T v0 = delta_ * v0_;
    T v1 = delta_ * ve_;
    if (t < 0) {
      return delta_ * v0;
    } else if (t < Tj1_) {  // case 1: 加速度上升
      return delta_ * (v0 + jm_ * pow(t, 2) / 2);
    } else if (t < Ta_ - Tj1_) {  // case 2: 加速度匀速
      return delta_ * (v0 + am_ * (t - Tj1_ / 2));
    } else if (t < Ta_) {  // case 3: 加速度减小
      return delta_ * (vm_ - jm_ * pow(Ta_ - t, 2) / 2);
    } else if (t < Ta_ + Tv_) {  // case 4: 加速度为0
      return delta_ * vm_;
    } else if (t < time_ - Td_ + Tj2_) {  // case 5: 减速度增大
      return delta_ * (vm_ - jm_ * pow(t - time_ + Td_, 2) / 2);
    } else if (t < time_ - Tj2_) {  // case 6: 减速度匀速
      return delta_ * (vm_ + dm_ * (t - time_ + Td_ - Tj2_ / 2));
    } else if (t < time_) {  // case 7: 减速度减小
      return delta_ * (v1 + jm_ * pow(time_ - t, 2) / 2);
    } else {  // case 8: 超过规划时间的情况
      return delta_ * v1;
    }
  }

  /**
   * @brief : Calculate the position
   * @param t : Current time_
   * @return : position
   */
  T x(const Real& t) const {
    T q0 = delta_ * x0_;
    T q1 = delta_ * xe_;
    T v0 = delta_ * v0_;
    T v1 = delta_ * ve_;
    if (t < 0) {
      return delta_ * q0;
    } else if (t < Tj1_) {  // case 1: 加速度上升
      return delta_ * (q0 + v0 * t + jm_ * pow(t, 3) / 6);
    } else if (t < Ta_ - Tj1_) {  // case 2: 加速度匀速
      return delta_ * (q0 + v0 * t + am_ / 6 * (3 * pow(t, 2) - 3 * Tj1_ * t + pow(Tj1_, 2)));
    } else if (t < Ta_) {  // case 3: 加速度减小
      return delta_ * (q0 + (vm_ + v0) * Ta_ / 2 - vm_ * (Ta_ - t) + jm_ * pow((Ta_ - t), 3) / 6);
    } else if (t < Ta_ + Tv_) {  // case 4: 加速度为0
      return delta_ * (q0 + (vm_ + v0) * Ta_ / 2 + vm_ * (t - Ta_));
    } else if (t < time_ - Td_ + Tj2_) {  // case 5: 减速度增大
      return delta_ * (q1 - (vm_ + v1) * Td_ / 2 + vm_ * (t - time_ + Td_) - jm_ * pow((t - time_ + Td_), 3) / 6);
    } else if (t < time_ - Tj2_) {  // case 6: 减速度匀速
      return delta_ * (q1 - (vm_ + v1) * Td_ / 2 + vm_ * (t - time_ + Td_) + dm_ / 6 * (3 * pow((t - time_ + Td_), 2) - 3 * Tj2_ * (t - time_ + Td_) + pow(Tj2_, 2)));
    } else if (t < time_) {  // case 7: 减速度减小
      return delta_ * (q1 - v1 * (time_ - t) - jm_ * pow((time_ - t), 3) / 6);
    } else {  // case 8: 超过规划时间的情况
      return delta_ * q1;
    }
  }
  T am_;  // Max acceleration
  T dm_;  // Max deceleration
  T jm_;
  T v0_;    // Initial velocity
  T ve_;    // End velocity
  T vm_;    // Max velocity
  T x0_;    // Initial position
  T xe_;    // End position
  T time_;  // Total time_

 protected:
 private:
  T ah_;  //
  T dh_;
  Real Tj1_;
  Real Tj2_;
  Real Ta_;
  Real Tv_;
  Real Td_;
  T delta_;
};
}  // namespace math
}  // namespace ul
#endif  // UL_SRC_UL_MATH_SSHAPEVELOCITY_H_
