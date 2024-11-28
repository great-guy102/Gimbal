/**
 * @file      pid_opt_core.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-27
 * @brief     积分优化器的核心实现，包括参数对齐、优化器基类
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
#ifndef HWCOMPONETS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_CORE_HPP_
#define HWCOMPONETS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_CORE_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"

namespace hello_world
{
namespace pid
{
/* Exported macro ------------------------------------------------------------*/
#ifndef __PID_PACKED_PARAMS
/**
 * @def __PID_PACKED_PARAMS
 * @brief
 * 定义一个宏, 用于设置结构体的属性为packed, 使得结构体的成员紧密排列,
 * 不会有编译器插入的填充字节
 * 相较于 aligned(4) 能够节省内存空间
 */
#define __PID_PACKED_PARAMS __attribute__((packed))
#endif /* __PID_PACKED_PARAMS */
/* Exported constants --------------------------------------------------------*/

enum PidOptState {
  kPidOptStateOk = 0u,                  ///< 正常
  kPidOptStateIllegalParams = 1u << 0,  ///< 参数非法
};
/* Exported types ------------------------------------------------------------*/
class __PID_PACKED_PARAMS PidOptimizer : public MemMgr
{
 public:
  typedef PidOptState State;

  PidOptimizer(void) = default;
  PidOptimizer(const PidOptimizer &) = default;
  PidOptimizer &operator=(const PidOptimizer &) = default;
  PidOptimizer(PidOptimizer &&) = default;
  PidOptimizer &operator=(PidOptimizer &&) = default;

  virtual ~PidOptimizer(void) = default;

  void enable(void) { enabled_ = true; }
  void disable(void) { enabled_ = false; }
  bool isEnabled(void) const { return enabled_; }
  bool isDisabled(void) const { return !enabled_; }

 protected:
  bool enabled_ = false;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace pid
}  // namespace hello_world

#endif /* HWCOMPONETS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_CORE_HPP_ */
