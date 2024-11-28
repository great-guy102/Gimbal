/**
 * @file      pid_opt_inte_separation.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-27
 * @brief     积分分离优化器的实现
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 *
 * @copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All Rights Reserved.
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2024-01-27 | ZhouShichan | 创建文件 |
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_INTE_SEPARATION_HPP_
#define HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_INTE_SEPARATION_HPP_

/* Includes ------------------------------------------------------------------*/
#include "pid_opt_core.hpp"
#include "system.hpp"

namespace hello_world
{
namespace pid
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
HW_OPTIMIZE_O2_START
/**
 * @class InteSeparation
 * @brief 积分分离优化器
 *
 * 积分环节的作用主要为消除静差，提高控制精度。
 * 但在短时间系统输出产生较大偏差时，可能会造成积分过度积累，使控制量过大，引起系统超调
 * 甚至振荡。
 * 此时需要引入积分分离，当被控量与给定值偏差较大时，取消积分作用；接近时引入积分控制。
 * - 具体计算公式见`calc`。
 * - 参数合法性检查见`isLegal`。
 */
class __PID_PACKED_PARAMS InteSeparation : public PidOptimizer
{
 public:
  InteSeparation(void) = default;
  /** 构造函数 @see setParams */
  InteSeparation(bool is_enabled, float lower, float upper)
  {
    setParams(is_enabled, lower, upper);
  }
  InteSeparation(const InteSeparation &) = default;
  InteSeparation &operator=(const InteSeparation &) = default;
  InteSeparation(InteSeparation &&) = default;
  InteSeparation &operator=(InteSeparation &&) = default;

  virtual ~InteSeparation(void) = default;

  /** 设置触发积分分离的误差值下限 */
  State setLower(float lower) { return setParams(enabled_, lower, upper_); }

  /** 触发积分分离的误差值上限 @see setParams */
  State setUpper(float upper) { return setParams(enabled_, lower_, upper); }

  float getLower(void) const { return lower_; }
  float getUpper(void) const { return upper_; }

  /** 当且仅当上限大于等于下限时，参数合法 */
  bool isLegal(void) { return lower_ <= upper_; }

  /**
   * @brief 设置积分分离优化参数
   * @param is_enabled 是否启用积分分离优化
   * @param lower 触发积分分离的误差值下限
   * @param upper 触发积分分离的误差值上限
   * @retval kPidOptStateOk 参数设置成功
   * @retval kPidOptStateIllegalParams 参数非法，已经自动处理
   * @see handleIllegalParams
   */
  State setParams(bool is_enabled, float lower, float upper)
  {
    is_enabled_ = is_enabled;
    lower_ = lower;
    upper_ = upper;
    return handleIllegalParams() ? kPidOptStateIllegalParams : kPidOptStateOk;
  }

  /**
   * @brief 检查和处理非法参数
   *
   * 若下限大于上限，则交换上下限，确保上限大于等于下限。
   * @retval true 参数非法，已经处理
   * @retval false 参数合法
   */
  bool handleIllegalParams(void)
  {
    if (lower_ > upper_) {
      float tmp = upper_;
      upper_ = lower_;
      lower_ = tmp;
      return true;
    } else {
      return false;
    }
  }

  /**
   * @brief 计算积分分离优化系数
   *
   * 当且仅当优化器使能、参数合法、误差值不在区间 [lower_,upper_) 时，返回0，否则返回1。
   * @details
   * 积分项表示为：$u_i(k)=\beta \sum_{j=t_0}^k{K_i e_i(j)}$。
   * 其中 $\beta$ 为积分分离系数，可由该类计算得到。
   * 具体的计算公式:
   * \f[
   * \beta=\begin{cases}
   * 0&,&isEnabled() \wedge e_i(k) \notin \left [\epsilon_l ,\epsilon_u \right]  \\
   * 1&,&others
   * \end{cases}
   * \f]，
   * 其中 \f$\epsilon_l\f$ 为参数中的下限值，\f$\epsilon_u\f$为参数中的上限值。
   * @param err 本次采样的误差值
   * @return 本次采样的积分分离优化系数
   * @retval 0 当 isEnabled() 且 e_i(k) 不在区间 [lower_,upper_]
   * @retval 1 其他情况
   */
  float calc(float err)
  {
    float ratio = 1;
    if (isEnabled() && isLegal()) {
      if (err < lower_ || err >= upper_) {
        ratio = 0;
      }
    }
    return ratio;
  }

 private:
  bool is_enabled_;  ///< 是否启用积分分离优化
  float lower_;      ///< 触发积分分离的误差值下限
  float upper_;      ///< 触发积分分离的误差值上限
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace pid
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_INTE_SEPARATION_HPP_ */
