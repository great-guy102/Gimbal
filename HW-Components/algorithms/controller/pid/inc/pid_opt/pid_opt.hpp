/**
 * @file      pid_opt.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-28
 * @brief
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
#ifndef HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_HPP_
#define HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_HPP_

/* Includes ------------------------------------------------------------------*/
// 输入优化
#include "pid_opt_setpoint_ramping.hpp"
// 误差计算优化
#include "pid_opt_dead_band.hpp"
#include "pid_opt_period_sub.hpp"
// 积分优化
#include "pid_opt_inte_anti_windup.hpp"
#include "pid_opt_inte_changing_rate.hpp"
#include "pid_opt_inte_separation.hpp"
#include "pid_opt_inte_trapezoidal.hpp"
// 差分优化
#include "pid_opt_diff_filter.hpp"
#include "pid_opt_diff_previous.hpp"
// 输出优化
#include "pid_opt_outlimit.hpp"

namespace hello_world
{
namespace pid
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace pid
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_HPP_ */
