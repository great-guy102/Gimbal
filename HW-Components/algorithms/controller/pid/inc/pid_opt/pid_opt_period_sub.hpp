/**
 * @file      pid_opt_period_sub.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-28
 * @brief     周期最小差值优化器的实现
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 *
 * @copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All Rights Reserved.
 *
 * @attention
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2024-MM-DD | ZhouShichan | description |
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_PERIOD_SUB_HPP_
#define HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_PERIOD_SUB_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>

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

class __PID_PACKED_PARAMS PeriodSub : public PidOptimizer
{
 public:
  PeriodSub(void) = default;
  /** 构造函数 @see setParams */
  PeriodSub(bool is_enabled, float period) { setParams(is_enabled, period); }
  PeriodSub(const PeriodSub &) = default;
  PeriodSub &operator=(const PeriodSub &) = default;
  PeriodSub(PeriodSub &&) = default;
  PeriodSub &operator=(PeriodSub &&) = default;

  virtual ~PeriodSub(void) = default;

  /** 设置周期值 @see setParams */
  State setPeriod(float period) { return setParams(enabled_, period); }

  /** 获取周期值 */
  float getPeriod(void) const { return period_; }

  /** 当且仅当周期值大于0.001时，参数合法 */
  bool isLegal(void) { return period_ > 0.001f; }

  /**
   * @brief 设置周期最小差值优化参数
   * @param is_enabled 是否启用周期最小差值优化
   * @param period 周期值
   * @retval kPidOptStateOk 参数设置成功
   * @retval kPidOptStateIllegalParams 参数非法，已经自动处理
   * @see handleIllegalParams
   */
  State setParams(bool is_enabled, float period)
  {
    enabled_ = is_enabled;
    period_ = period;
    return handleIllegalParams() ? kPidOptStateIllegalParams : kPidOptStateOk;
  }
  /**
   * @brief 检查和处理非法参数
   *
   * 若周期值小于0.001，则将周期值置为0.001。
   * @retval true 参数非法，已经处理
   * @retval false 参数合法
   */
  bool handleIllegalParams(void)
  {
    if (period_ < 0.001f) {
      period_ = 0.001f;
      return true;
    } else {
      return false;
    }
  }
  /**
   * @brief 计算周期最小差值优化后的误差值
   *
   * @param ref 参考值
   * @param fdb 反馈值
   * @return 循环周期数据中ref和fdb最小的误差值
   */
  float calc(float ref, float fdb)
  {
    if (isEnabled() && isLegal()) {
      float delta = ref - fdb;
      float times = roundf(delta / period_);
      return delta - times * period_;
    } else {
      return ref - fdb;
    }
  }

 private:
  float period_ = 0.0f;  ///< 周期值
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace pid
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_PERIOD_SUB_HPP_ */
