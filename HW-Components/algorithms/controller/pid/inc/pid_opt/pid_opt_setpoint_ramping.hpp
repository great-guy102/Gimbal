/**
 * @file      pid_opt_setpoint_ramping.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-27
 * @brief     给定值斜坡优化器的实现
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
#ifndef HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_SETPOINT_RAMPING_HPP_
#define HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_SETPOINT_RAMPING_HPP_

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
 * @class SetpointRamping
 * @brief 设定点斜坡优化器
 *
 * 当给定值出现较大的阶跃变化，很容易引起超调。
 * 使用线性斜坡函数或一阶滤波为给定值安排过渡过程，使其从其旧值逐渐变化到新值。
 * 这样可以避免阶跃变化产生的不连续性，进而使对象运行平稳，适用于高精度伺服系统的位置跟
 * 踪。
 * - 具体计算公式见`calc`。
 * - 参数合法性检查见`isLegal`。
 */
class __PID_PACKED_PARAMS SetpointRamping : public PidOptimizer
{
 public:
  SetpointRamping(void) = default;

  /** 构造函数 @see setParams  @see calc */
  SetpointRamping(bool is_enabled, float lower, float upper, float weight)
  {
    setParams(is_enabled, lower, upper, weight);
  }
  /** 设置触发一阶滤波的区间下界 @see setParams */
  State setLower(float lower)
  {
    return setParams(enabled_, lower, upper_, weight_);
  }
  /** 设置触发一阶滤波的区间上界 @see setParams */
  State setUpper(float upper)
  {
    return setParams(enabled_, lower_, upper, weight_);
  }
  /** 设置进行一阶滤波的权重系数 @see setParams */
  State setWeight(float weight)
  {
    return setParams(enabled_, lower_, upper_, weight);
  }

  float getLower(void) const { return lower_; }
  float getUpper(void) const { return upper_; }
  float getWeight(void) const { return weight_; }

  bool isLegal(void) { return lower_ < upper_ && 0 <= weight_ && weight_ <= 1; }

  /**
   * @brief 设置设定点斜坡优化参数
   * @param is_enabled 是否启用设定点斜坡优化
   * @param lower 触发给定值平滑滤波的区间下界
   * @param upper 触发给定值平滑滤波的区间上界
   * @param weight 对给定值进行低通滤波的权重值
   * @retval kPidOptStateOk 参数设置成功
   * @retval kPidOptStateIllegalParams 参数非法，已经自动处理
   * @see handleIllegalParams
   */
  State setParams(bool is_enabled, float lower, float upper, float weight)
  {
    enabled_ = is_enabled;
    lower_ = lower;
    upper_ = upper;
    weight_ = weight;
    return handleIllegalParams() ? kPidOptStateIllegalParams : kPidOptStateOk;
  }

  /**
   * @brief 检查并处理非法参数
   *
   * 若下限大于上限，则交换上下限，确保上限大于等于下限。
   * 若权重值小于0，则将权重值设为0，若权重值大于1，则将权重值设为1，确保权重值在 [0,1]
   * 区间之上。
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
    if (weight_ < 0) {
      weight_ = 0;
      with_illegal_params = true;
    } else if (weight_ > 1) {
      weight_ = 1;
      with_illegal_params = true;
    }
    return with_illegal_params;
  }

  /**
   * @brief 计算设定点斜坡优化后的输出值
   *
   * 当且仅当优化器使能、参数合法、参考值不在区间 [lower_,upper_) 时，对本次参考值进行
   * 一阶滤波，其中上次采样的参考值权重为参数中的权重值；否则，不进行滤波，返回本次采样
   * 的参考值。
   * @details 具体的计算公式为：
   * \f[
   * r(k)=\begin{cases}
   * w*r(k-1)+(1-w)*r(k)  & ,  isEnabled() \wedge r(j) \notin \left [ \epsilon_l,\epsilon_u \right ]   \\
   * r(k)  & ,others
   * \end{cases}
   * \f]，
   *  其中 \f$\epsilon_l\f$ 为参数中的下限值，
   * \f$\epsilon_u\f$为参数中的上限值，
   * \f$w\f$为参数中的权重值
   * @param ref 本次采样的参考值
   * @param last_ref 上次采样的参考值
   * @return 本次采样的设定点斜坡优化后的输出值
   */
  float calc(float ref, float last_ref)
  {
    float res = ref;
    if (isEnabled()) {
      float delta_err = ref - last_ref;
      if (delta_err <= lower_ || delta_err >= upper_) {
        res = weight_ * last_ref + (1 - weight_) * ref;
      }
    }
    return res;
  }

 private:
  float lower_ = 0;   ///< 触发设定点斜坡优化的参考值下限
  float upper_ = 0;   ///< 触发设定点斜坡优化的参考值上限
  float weight_ = 0;  ///< 计算设定点斜坡优化的权重值
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace pid
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_SETPOINT_RAMPING_HPP_ */
