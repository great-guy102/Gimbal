/**
 *******************************************************************************
 * @file      : controller_base.hpp
 * @brief     : 控制器基类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-25      Caikunzhen      1. 未测试版本
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_CONTROLLER_BASE_CONTROLLER_BASE_HPP_
#define HW_COMPONENTS_ALGORITHMS_CONTROLLER_BASE_CONTROLLER_BASE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstddef>

#include "allocator.hpp"
namespace hello_world
{

/* Exported macro ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
enum ControllerState {
  kControllerStateOk = 0u,
  kControllerStateError = 1u << 0,
};

class Controller : public MemMgr
{
 public:
  typedef ControllerState State;

  explicit Controller(void) = default;
  /**
   * @brief       构造函数
   * @param        ref_dim: 参考值维度，> 0
   * @param        fdb_dim: 反馈值维度，> 0
   * @param        out_dim: 输出值维度，> 0
   * @param        ffd_dim: 前馈值维度，>= 0
   * @retval       None
   * @note        None
   */
  Controller(size_t ref_dim, size_t fdb_dim, size_t out_dim, size_t ffd_dim = 0)
      : ref_dim_(ref_dim),
        fdb_dim_(fdb_dim),
        ffd_dim_(ffd_dim),
        out_dim_(out_dim) {};
  Controller(const Controller&) = default;
  Controller& operator=(const Controller&) = default;
  Controller(Controller&&) = default;
  Controller& operator=(Controller&&) = default;

  virtual ~Controller(void) = default;

  /**
   * @brief       计算
   * @param        ref_arr: 参考值数组
   * @param        fdb_arr: 反馈值数组
   * @param        ffd_arr: 前馈值数组
   * @param        out_arr: 输出值数组
   * @retval       控制器状态，可能的值有：
   *   @arg        kControllerStateOk: 正常
   *   @arg        kControllerStateError: 错误
   * @note        None
   */
  virtual State calc(const float ref_arr[], const float fdb_arr[],
                     const float ffd_arr[], float out_arr[]) = 0;

  /**
   * @brief       重置
   * @param        None
   * @retval       None
   * @note        None
   */
  virtual State reset(void) = 0;

 protected:
  size_t ref_dim_ = 0;
  size_t fdb_dim_ = 0;
  size_t ffd_dim_ = 0;
  size_t out_dim_ = 0;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_CONTROLLER_BASE_CONTROLLER_BASE_HPP_ */
