/**
 * @file      pid_opt_inte_changing_rate.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-27
 * @brief     变速积分优化器的实现
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
#ifndef HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_INTE_CHANGING_RATE_HPP_
#define HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_INTE_CHANGING_RATE_HPP_

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
 * @class InteChangingRate
 * @brief 变速积分优化器
 *
 * 基于积分分离优化的思想，根据系统偏差大小改变积分的速度，偏差越大，积分越慢，反之则越
 * 快，形成连续的变化过程。
 * 控制器参数类用于计算变速积分比例系数，根据给定的上下限和使能状态，计算比例系数。
 * - 具体计算公式见`calc`。
 * - 参数合法性检查见`isLegal`。
 */
class __PID_PACKED_PARAMS InteChangingRate : public PidOptimizer
{
 public:
  InteChangingRate(void) = default;
  /** 构造函数 @see setParams */
  InteChangingRate(bool is_enabled, float lower, float upper)
  {
    setParams(is_enabled, lower, upper);
  }
  InteChangingRate(const InteChangingRate &) = default;
  InteChangingRate &operator=(const InteChangingRate &) = default;
  InteChangingRate(InteChangingRate &&) = default;
  InteChangingRate &operator=(InteChangingRate &&) = default;

  virtual ~InteChangingRate(void) = default;

  /** 设置下限 @see setParams */
  inline State setLower(float lower)
  {
    return setParams(enabled_, lower, upper_);
  }
  /** 设置上限 @see setParams */
  inline State setUpper(float upper)
  {
    return setParams(enabled_, lower_, upper_);
  }

  /** 获取下限 */
  float getLower(void) const { return lower_; }

  /** 获取上限 */
  float getUpper(void) const { return upper_; }

  /** 当且仅当下限大于等于 0 且上限大于下限时，参数合法 */
  bool isLegal(void) { return 0 <= lower_ && lower_ < upper_; }

  /**
   * @brief 设置参数
   * @param is_enabled 是否使能
   * @param lower 小限
   * @param upper 大限
   * @retval kPidOptStateOk 参数设置成功
   * @retval kPidOptStateIllegalParams 参数非法，已经自动处理
   */
  State setParams(bool is_enabled, float lower, float upper)
  {
    enabled_ = is_enabled;
    lower_ = lower;
    upper_ = upper;
    return handleIllegalParams() ? kPidOptStateIllegalParams : kPidOptStateOk;
  }

  /**
   * @brief 检查和处理非法参数
   *
   * 若下限小于0，则将下限设置为0。
   * 若上限小于下限，则将上限设置为下限加上一个较小的值。
   * @retval true 参数非法，已经处理
   * @retval false 参数合法
   */
  bool handleIllegalParams(void)
  {
    bool with_illegal_params = false;
    if (lower_ < 0) {
      lower_ = 0;
      with_illegal_params = true;
    }

    if (lower_ > upper_) {
      upper_ = lower_ + 0.001f;
      with_illegal_params = true;
    }
    return with_illegal_params;
  }

  /**
   * @brief 计算变速积分系数
   *
   * 当且仅当优化器使能、参数合法时，进行计算：如果误差值超过上限，则比例系数为0；如果
   * 误差值在上下限之间，则根据误差值的大小计算比例系数；如果小于等于下限，则返回 1。
   * 不计算时，返回 1。
   * @details
   * 在积分项计算公式中引入变速积分比例系数 \f$f[e_i(k)]\f$。
   * 此时积分项表示为 \f$ u_i(k)=\sum_{j=0}^{k-1}K_i e_i(j) T(j)+ f[e_i(k)]K_i e_i(k) T(k) \f$
   * 计算公式为：
   * \f[
   * f[e(k)]=\begin{cases}
   * 0&,& isEnabled() \wedge |e(k)| \in [\epsilon_u,+\inf)\\
   * \frac{\epsilon_u- |e(k)| }{\epsilon_u-\epsilon_l}&,&isEnabled() \wedge |e(k)| \in [\epsilon_l,\epsilon_u)\\
   * 1&,& others
   * \end{cases}
   * \f],
   * 其中 \f$\epsilon_l\f$ 为参数中的下限值，\f$\epsilon_u\f$为参数中的上限值。
   * @param err 误差值
   * @return 变速积分系数
   */
  float calc(float err)
  {
    float ratio = 1;
    if (isEnabled() && isLegal()) {
      float fabs_err = err < 0 ? -err : err;
      if (fabs_err >= upper_) {
        ratio = 0;
      } else if (fabs_err > lower_) {
        ratio = (upper_ - fabs_err) / (upper_ - lower_);
      }
    }
    return ratio;
  }

 private:
  float lower_ = 0;  ///< 下限，应该大于等于 0
  float upper_ = 0;  ///< 上限，应该大于 lower_
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace pid
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_INTE_CHANGING_RATE_HPP_ */
