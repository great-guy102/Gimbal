/**
 *******************************************************************************
 * @file      : fsm.hpp
 * @brief     : 有限状态机
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_FSM_FSM_HPP_
#define HW_COMPONENTS_ALGORITHMS_FSM_FSM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <functional>

#include "allocator.hpp"

namespace hello_world
{
namespace fsm
{
/* Exported macro ------------------------------------------------------------*/
enum class ReturnState {
  kOk = 0x0,
  kStateOutOfRange,  ///< 状态超出范围
  kChangeState,      ///< 状态改变
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class FsmBase : MemMgr
{
 public:
  typedef size_t State;

  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  FsmBase(void) = default;
  /**
   * @brief       状态机初始化
   * @param        state_num: 状态数量
   * @param        update_func: 状态更新函数，在 update 中自动调用
   * @note        None
   */
  FsmBase(size_t state_num, std::function<State(void)> update_func);
  FsmBase(const FsmBase& other);
  FsmBase& operator=(const FsmBase& other);
  FsmBase(FsmBase&& other);
  FsmBase& operator=(FsmBase&& other);

  virtual ~FsmBase(void);

  /* 配置方法 */

  /**
   * @brief       状态机初始化，使用默认构造函数后请务必调用此函数
   * @param        state_num: 状态数量
   * @param        update_func: 状态更新函数，在 update 中自动调用
   * @note        None
   */
  void init(size_t state_num, std::function<State(void)> update_func);

  /* 功能性方法 */

  /**
   * @brief       状态机状态更新
   * @retval       更新结果
   *   @arg        ReturnState::kOk: 正常运行
   *   @arg        ReturnState::kStateOutOfRange: 状态超出范围
   *   @arg        ReturnState::kChangeState: 状态改变
   * @note        None
   */
  ReturnState update(void);

  /**
   * @brief       状态机运行
   * @param        None
   * @retval       运行结果
   *   @arg        ReturnState::kOk: 正常运行
   * @note        None
   */
  ReturnState run(void);

  /**
   * @brief       状态机注册状态
   * @param        state: 状态
   * @param        func: 回调函数，在 run 中自动调用
   * @retval       注册结果
   *   @arg        ReturnState::kOk: 正常运行
   *   @arg        ReturnState::kStateOutOfRange: 状态超出范围
   * @note        状态超出范围时不会注册
   */
  ReturnState registerStateFunc(State state, std::function<void(void)> func);

  /* 数据修改与获取 */

  State state(void) const { return state_; }

  size_t state_num(void) const { return state_num_; }

 private:
  State state_ = 0;       ///< 当前状态
  size_t state_num_ = 0;  ///< 状态数量

  std::function<State(void)> update_func_;  ///< 状态更新函数

  std::function<void(void)>* state_func_ls_ = nullptr;  ///< 状态回调函数列表
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace fsm
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_FSM_FSM_HPP_ */
