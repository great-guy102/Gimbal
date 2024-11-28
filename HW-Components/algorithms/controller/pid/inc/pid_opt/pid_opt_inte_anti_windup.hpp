/**
 * @file      pid_opt_inte_anti_windup.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-27
 * @brief     积分抗饱和优化器的实现
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
#ifndef HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_INTE_ANTI_WINDUP_HPP_
#define HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_INTE_ANTI_WINDUP_HPP_

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
 * @class InteAntiWindup
 * @brief 积分抗饱和优化器
 *
 * 该类的作用是提供积分抗饱和优化的实现，以解决控制器输出由于积分作用导致的饱和现象，从
 * 而提高控制性能该类的主要功能包括：
 * 计算抗积分饱和系数 alpha，根据是否启用该功能以及上一时刻的输出和当前时刻的误差值是否
 * 满足特定条件来决定 alpha 的值。
 * - 具体计算公式见`calc`。
 * - 参数合法性检查见`isLegal`。
 * - 启用判定见`isEnabled`。
 */
class __PID_PACKED_PARAMS InteAntiWindup : public PidOptimizer
{
 public:
  InteAntiWindup(void) = default;
  /** 构造函数 @see setParams */
  InteAntiWindup(bool is_enabled, float lower, float upper)
  {
    setParams(is_enabled, lower, upper);
  }
  InteAntiWindup(const InteAntiWindup &) = default;
  InteAntiWindup &operator=(const InteAntiWindup &) = default;
  InteAntiWindup(InteAntiWindup &&) = default;
  InteAntiWindup &operator=(InteAntiWindup &&) = default;

  virtual ~InteAntiWindup(void) = default;

  /** 设置触发积分抗饱和的积分输出值下限 @see setParams */
  State setLower(float lower) { return setParams(isEnabled(), lower, upper_); }

  /** 设置触发积分抗饱和的积分输出值上限 @see setParams */
  State setUpper(float upper) { return setParams(isEnabled(), lower_, upper); }

  /** 获取触发积分抗饱和的积分输出值下限 */
  float getLower(void) const { return lower_; }

  /** 获取触发积分抗饱和的积分输出值上限 */
  float getUpper(void) const { return upper_; }

  /** 当且仅当参数下限小于等于参数上限时，参数合法 */
  bool isLegal(void) const { return lower_ <= upper_; }

  /**
   * @brief 设置积分抗饱和优化参数
   * @param is_enabled 是否启用积分抗饱和优化
   * @param lower 触发积分抗饱和的积分输出值下限
   * @param upper 触发积分抗饱和的积分输出值上限
   * @retval kPidOptStateOk 参数合法且设置成功
   * @retval kPidOptStateIllegalParams 参数不合法，自动修改后设置成功
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
   * 如果参数下限大于上限，则自动交换两个参数的值。
   * @retval true 参数非法，已经自动处理
   * @retval false 参数合法
   */
  bool handleIllegalParams(void)
  {
    bool with_illegal_params = false;
    if (lower_ > upper_) {
      float tmp = upper_;
      upper_ = lower_;
      lower_ = tmp;
      with_illegal_params = true;
    }
    return with_illegal_params;
  }

  /**
   * @brief 计算积分抗饱和优化系数
   *
   * 当且仅当优化器使能、参数合法、此次误差会积分继续累计且上一次的积分输出值超出了上下
   * 限时，返回0；否则，返回1。
   * @details 具体计算公式为：
   * \f[
   * \alpha=\begin{cases}
   * 0 &,&isEnabled() \wedge  \left [ u(k-1)\cdot e(k)>0 \vee  u(k-1)\notin \left [\epsilon_l ,\epsilon_u \right] \right ] \\
   * 1 &,&others
   * \end{cases}
   * \f]，
   * 其中 \f$\epsilon_l\f$ 为参数中的下限值，\f$\epsilon_u\f$为参数中的上限值。
   * @param err 本次采样的误差值，仅符号参与计算
   * @param last_out 上次采样的输出值
   * @return 本次采样的积分抗饱和优化系数
   * @retval 0 优化器使能、参数合法、此次误差会积分继续累计且上一次的积分输出值超出了上
   *         下限
   * @retval 1 其他情况，正常累积此次误差
   */
  float calc(float err, float last_inte_out)
  {
    float ratio = 1;
    if (isEnabled() && isLegal()) {
      if (err * last_inte_out > 0 &&
          (last_inte_out < lower_ || last_inte_out > upper_)) {
        ratio = 0;
      }
    }
    return ratio;
  }

 private:
  float lower_ = 0.0f;  ///< 触发积分抗饱和的积分输出值下限
  float upper_ = 0.0f;  ///< 触发积分抗饱和的积分输出值上限
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace pid
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_INTE_ANTI_WINDUP_HPP_ */
