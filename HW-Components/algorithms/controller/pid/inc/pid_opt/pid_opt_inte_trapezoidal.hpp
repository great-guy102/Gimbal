/**
 * @file      pid_opt_inte_trapezoidal.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-27
 * @brief     梯形积分优化器的实现
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
 * | 1.0.0 | 2024-01-27 | ZhouShichan | 创建文件 |
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_INTE_TRAPEZOIDAL_HPP_
#define HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_INTE_TRAPEZOIDAL_HPP_

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
 * @class InteTrapezoidal
 * @brief 梯形积分优化器
 *
 * 为避免由于积分环节的作用而引起的系统超调甚至振荡，可采用梯形积分算法。
 * 梯形积分算法是将当前时刻和上一时刻的积分值加权平均，从而减小积分环节的作用，提高系统
 * 的动态特性。
 * - 具体计算公式见`calc`
 * - 参数合法性检查见`isLegal`
 */
class __PID_PACKED_PARAMS InteTrapezoidal : public PidOptimizer
{
 public:
  InteTrapezoidal(void) = default;
  /** 构造函数 @see setParams */
  explicit InteTrapezoidal(bool is_enabled) { setParams(is_enabled); }
  InteTrapezoidal(const InteTrapezoidal &) = default;
  InteTrapezoidal &operator=(const InteTrapezoidal &) = default;
  InteTrapezoidal(InteTrapezoidal &&) = default;
  InteTrapezoidal &operator=(InteTrapezoidal &&) = default;

  virtual ~InteTrapezoidal(void) = default;

  /**
   * 设置积分分离优化参数
   * @retval kPidOptStateOk 参数设置成功
   */
  State setParams(bool is_enabled)
  {
    enabled_ = is_enabled;
    return kPidOptStateOk;
  }

  /** 参数始终合法 */
  bool isLegal(void) { return true; }

  /**
   * @brief 计算梯形积分优化后的积分误差值
   *
   * 当且仅当优化器使能、参数合法时，返回两次采样误差的平均值；否则，返回本次采样的误差
   * 值。
   * @details
   * 具体计算公式为：
   * \f[
   * e_i(k)=\begin{cases}
   * \left ( e(k)+e(k-1) \right )/2&\text{if} \quad isEnabled()\\
   * e(k)& others
   * \end{cases}
   * \f]
   * @param err 本次采样的误差值
   * @param last_err 上次采样的误差值
   * @return 本次采样用于积分的误差值
   */
  float calc(float err, float last_err)
  {
    float inte_err = err;
    if (isEnabled()) {
      inte_err = (err + last_err) / 2;
    }
    return inte_err;
  }
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace pid
}  // namespace hello_world

#endif /*  HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_INTE_TRAPEZOIDAL_HPP_ */
