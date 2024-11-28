/**
 * @file      pid_opt_dead_band.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-27
 * @brief     死区优化器的实现
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
#ifndef HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_DEAD_BAND_HPP_
#define HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_DEAD_BAND_HPP_

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
 * @class DeadBand
 * @brief 死区优化器
 *
 * 为避免控制作用过于频繁，消除由于频繁动作所引起的振荡，必要时可采用带死区的 PID 控制
 * 算法。
 * 判断控制偏差是否小于给定阈值，若小于则输入误差为0。
 * - 具体计算公式见`calc`。
 * - 参数合法性检查见`isLegal`。
 */
class __PID_PACKED_PARAMS DeadBand : public PidOptimizer
{
 public:
  DeadBand(void) = default;
  /** 带参构造函数 @see setParams */
  DeadBand(bool is_enabled, float lower, float upper)
  {
    setParams(is_enabled, lower, upper);
  }
  DeadBand(const DeadBand &) = default;
  DeadBand &operator=(const DeadBand &) = default;
  DeadBand(DeadBand &&) = default;
  DeadBand &operator=(DeadBand &&) = default;

  virtual ~DeadBand(void) = default;

  /** 设置死区下限 @see setParams */
  State setLower(float lower) { return setParams(enabled_, lower, upper_); }

  /** 设置死区上限 @see setParams */
  State setUpper(float upper) { return setParams(enabled_, lower_, upper_); }

  /** 获取死区下限 */
  float getLower(void) const { return lower_; }

  /** 获取死区上限 */
  float getUpper(void) const { return upper_; }

  /** 当且仅当死区下限小于等于上限时，参数合法 */
  bool isLegal(void) const { return lower_ <= upper_; }

  /**
   * @brief 设置死区优化参数
   * @param is_enabled 是否启用死区优化
   * @param lower 死区下限
   * @param upper 死区上限
   * @retval kPidOptStateOk 参数设置成功
   * @retval kPidOptStateIllegalParams 参数非法，已经自动处理
   * @see handleIllegalParams
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
   * 若下限大于上限，则交换上下限，确保上限大于等于下限。
   * @retval true 参数非法，已经处理
   * @retval false 参数合法
   */
  bool handleIllegalParams(void)
  {
    bool with_illegal_params = false;
    if (!(lower_ <= upper_)) {
      float tmp = upper_;
      upper_ = lower_;
      lower_ = tmp;
      with_illegal_params = true;
    }
    return with_illegal_params;
  }

  /**
   * @brief 计算死区优化后的误差值
   *
   * 当优化器使能且参数合法时，若误差值大于上限，则优化后的误差值为原误差值减去上限；
   * 若误差值小于下限，则优化后的误差值为原误差值减去下限；否则优化后的误差值为 0。
   * 否则，不进行死区优化，直接返回输入。
   * @details 具体的计算公式为：
   * \f\f]
   * e(k)=\begin{cases}
   * e(k)-\epsilon_l  & , isEnabled() \wedge e(k) \in (-\inf,\epsilon_l)  \\
   * 0&, isEnabeld() \wedge e(k) \in [\epsilon_l,\epsilon_u]\\
   * e(k)-\epsilon_u  & , isEnabled() \wedge e(k) \in (\epsilon_u,+\inf ) \\
   * e(k) & ,others,
   * \end{cases}
   * 其中 \f$\epsilon_l\f$ 为参数中的下限值，
   * \f$\epsilon_u\f$为参数中的上限值。
   * @param err 本次采样的误差值
   * @return 本次采样死区优化后的误差值
   */
  float calc(float err)
  {
    float res = err;
    if (isEnabled() && isLegal()) {
      if (lower_ <= err && err <= upper_) {
        float delta = upper_ - lower_;
        float l = lower_ + delta / 10;
        float u = upper_ - delta / 10;
        if (err > u) {
          res -= (res - u) / 5;
        } else if (err < l) {
          res += (l - res) / 5;
        } else {
          res = (lower_ + upper_) / 2;
        }
      }
    }
    return res;
  }

 private:
  float lower_ = 0.0f;  ///< 死区下限
  float upper_ = 0.0f;  ///< 死区上限
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace pid
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_DEAD_BAND_HPP_ */
