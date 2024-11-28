/**
 * @file      pid_opt_diff_filter.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-27
 * @brief     微分滤波优化器的实现
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 * @copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All Rights Reserved.
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2024-01-27 | ZhouShichan | 创建文件 |
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_DIFF_FILTER_HPP_
#define HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_DIFF_FILTER_HPP_

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
 * @class DiffFilter
 * @brief 差分滤波器
 *
 * 差分滤波器用于对输入信号进行滤波处理，根据触发滤波的差分值范围和权重进行滤波操作。
 * 微分项可改善系统的动态特性，但也易引进高频干扰，在误差扰动突变时尤其显出微分项的不足。
 * 因此，可以在控制算法中加入一阶惯性环节（低通滤波器），可使系统性能得到改善。
 * 可将滤波器直接加在微分环节上或控制器输出上。
 * 此方案也称为“不完全微分”。
 * 事实上，还可以针对系统的频域特性设计合适的滤波器，这里不做更深入的探究。
 * - 具体计算公式见`calc`。
 * - 参数合法性检查见`isLegal`。
 */
class __PID_PACKED_PARAMS DiffFilter : public PidOptimizer
{
 public:
  DiffFilter(void) = default;
  /** 构造函数 @see setParams */
  DiffFilter(bool is_enabled, float lower, float upper, float weight)
  {
    setParams(is_enabled, lower, upper, weight);
  }
  DiffFilter(const DiffFilter &) = default;
  DiffFilter &operator=(const DiffFilter &) = default;
  DiffFilter(DiffFilter &&) = default;
  DiffFilter &operator=(DiffFilter &&) = default;

  /** 设置差分滤波器优化的触发下限 @see setParams */
  State setLower(float lower)
  {
    return setParams(enabled_, lower, upper_, weight_);
  }
  /** 设置差分滤波器优化的触发上限 @see setParams */
  State setUpper(float upper)
  {
    return setParams(enabled_, lower_, upper, weight_);
  }
  /** 设置差分滤波器的低通滤波权重 @see setParams */
  State setWeight(float weight)
  {
    return setParams(enabled_, lower_, upper_, weight);
  }

  /** 获取差分滤波器的触发下限 */
  float getLower(void) const { return lower_; }
  /** 获取差分滤波器的触发上限 */
  float getUpper(void) const { return upper_; }
  /** 获取差分滤波器的低通滤波权重 */
  float getWeight(void) const { return weight_; }

  /** 当且仅当参数下限小于等于参数上限，且权重值在[0,1]上时，参数合法 */
  bool isLegal(void)
  {
    return lower_ <= upper_ && 0 <= weight_ && weight_ <= 1;
  }

  /**
   * @brief 设置差分滤波器的参数
   *
   * @param is_enabled 是否启用差分滤波器
   * @param lower 触发滤波的差分值下限
   * @param upper 触发滤波的差分值上限
   * @param weight 低通滤波的权重值
   * @retval kPidOptStateOk 参数合法，设置成功
   * @retval kPidOptStateIllegalParams 参数非法，自动修改后设置成功
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
   * @brief 检查和处理非法参数
   *
   * 若下限大于上限，则交换上下限，确保上限大于等于下限。
   * 若权重值小于0，则置为0；若权重值大于1，则置为1，确保权重在 [0,1] 区间上。
   * @retval true 参数非法，已经处理
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
    }
    if (weight_ > 1) {
      weight_ = 1;
      with_illegal_params = true;
    }
    return with_illegal_params;
  }

  /**
   * @brief 计算差分滤波器的输出值
   *
   * 当且仅当优化器使能、参数合法、此次差分值与上次差分值的差值超过给定的阈值上下限时，
   * 进行低通滤波操作，其中 last_diff_out 的权重为参数给定的权重值；否则，不进行滤波操
   * 作。
   * @details 具体计算公式：
   * \f[
   * u_d(k)=\begin{cases}
   * w*u_d(k-1)+(1-w)*u_d(k)  & ,  isEnabled() \wedge u_d(j) \notin \left [ \epsilon_l,\epsilon_u \right ]   \\
   * u_d(k)  & ,others
   * \end{cases}
   * \f]，
   * 其中 \f$\epsilon_l\f$ 为参数中的下限值，
   * \f$\epsilon_u\f$为参数中的上限值，
   * \f$w\f$为参数中的权重值
   * @param diff_out 当前差分值
   * @param last_diff_out 上一次的差分值
   * @return 计算后的滤波结果
   */
  float calc(float diff_out, float last_diff_out)
  {
    float res = diff_out;
    if (isEnabled() && isLegal()) {
      float delta_diff_out = diff_out - last_diff_out;
      if (delta_diff_out < lower_ || delta_diff_out >= upper_) {
        res = weight_ * last_diff_out + (1 - weight_) * diff_out;
      }
    }
    return res;
  }

 private:
  float lower_ = 0.0f;   ///< 触发滤波的触发滤波的差分值下限
  float upper_ = 0.0f;   ///< 触发滤波的触发滤波的差分值上限
  float weight_ = 0.0f;  ///< 滤波的低通滤波的权重值
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace pid
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_DIFF_FILTER_HPP_ */
