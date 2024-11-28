/**
 *******************************************************************************
 * @file      : fsm.cpp
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
/* Includes ------------------------------------------------------------------*/
#include "fsm.hpp"

#include "assert.hpp"
#include "construct.hpp"

namespace hello_world
{
namespace fsm
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

FsmBase::FsmBase(size_t state_num, std::function<State(void)> update_func)
    : state_num_(state_num), update_func_(update_func)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(state_num > 0, "state_num must be greater than 0");
#pragma endregion

  state_ = 0;

  state_func_ls_ =
      Allocator<std::function<void(void)>>::allocate(state_num_);
  for (size_t i = 0; i < state_num_; i++) {
    Allocator<std::function<void(void)>>::construct(state_func_ls_ + i);
  }
}

FsmBase::FsmBase(const FsmBase& other)
{
  state_num_ = other.state_num_;
  state_ = other.state_;
  update_func_ = other.update_func_;

  state_func_ls_ =
      Allocator<std::function<void(void)>>::allocate(state_num_);
  for (size_t i = 0; i < state_num_; i++) {
    Allocator<std::function<void(void)>>::construct(state_func_ls_ + i,
                                                    other.state_func_ls_[i]);
  }
}

FsmBase& FsmBase::operator=(const FsmBase& other)
{
  if (this != &other) {
    if (state_func_ls_ != nullptr) {
      for (size_t i = 0; i < state_num_; i++) {
        Allocator<std::function<void(void)>>::destroy(state_func_ls_ + i);
      }
      Allocator<std::function<void(void)>>::deallocate(
          state_func_ls_, state_num_);
    }

    state_num_ = other.state_num_;
    state_ = other.state_;
    update_func_ = other.update_func_;

    state_func_ls_ =
        Allocator<std::function<void(void)>>::allocate(state_num_);
    for (size_t i = 0; i < state_num_; i++) {
      Allocator<std::function<void(void)>>::construct(state_func_ls_ + i,
                                                      other.state_func_ls_[i]);
    }
  }
  return *this;
}

FsmBase::FsmBase(FsmBase&& other)
{
  state_num_ = other.state_num_;
  state_ = other.state_;
  update_func_ = std::move(other.update_func_);
  state_func_ls_ = other.state_func_ls_;

  other.state_func_ls_ = nullptr;
}

FsmBase& FsmBase::operator=(FsmBase&& other)
{
  if (this != &other) {
    if (state_func_ls_ != nullptr) {
      for (size_t i = 0; i < state_num_; i++) {
        Allocator<std::function<void(void)>>::destroy(state_func_ls_ + i);
      }
      Allocator<std::function<void(void)>>::deallocate(
          state_func_ls_, state_num_);
    }

    state_num_ = other.state_num_;
    state_ = other.state_;
    update_func_ = std::move(other.update_func_);
    state_func_ls_ = other.state_func_ls_;

    other.state_func_ls_ = nullptr;
  }
  return *this;
}

FsmBase::~FsmBase(void)
{
  for (size_t i = 0; i < state_num_; i++) {
    Allocator<std::function<void(void)>>::destroy(state_func_ls_ + i);
  }
  Allocator<std::function<void(void)>>::deallocate(state_func_ls_, state_num_);
}

void FsmBase::init(size_t state_num, std::function<State(void)> update_func)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(state_num > 0, "state_num must be greater than 0");
#pragma endregion

  if (state_func_ls_ != nullptr) {
    for (size_t i = 0; i < state_num_; i++) {
      Allocator<std::function<void(void)>>::destroy(state_func_ls_ + i);
    }
    Allocator<std::function<void(void)>>::deallocate(
        state_func_ls_, state_num_);
  }

  state_ = 0;
  state_num_ = state_num;
  update_func_ = update_func;

  state_func_ls_ =
      Allocator<std::function<void(void)>>::allocate(state_num_);
  for (size_t i = 0; i < state_num_; i++) {
    Allocator<std::function<void(void)>>::construct(state_func_ls_ + i);
  }
}

ReturnState FsmBase::update(void)
{
  State state = update_func_();

  if (state < state_num_) {
    if (state != state_) {
      state_ = state;
      return ReturnState::kChangeState;
    } else {
      return ReturnState::kOk;
    }
  } else {
    return ReturnState::kStateOutOfRange;
  }
}

ReturnState FsmBase::run(void)
{
  state_func_ls_[state_]();
  return ReturnState::kOk;
}

ReturnState FsmBase::registerStateFunc(
    State state, std::function<void(void)> func)
{
  if (state < state_num_) {
    state_func_ls_[state] = func;
    return ReturnState::kOk;
  } else {
    return ReturnState::kStateOutOfRange;
  }
}
/* Private function definitions ----------------------------------------------*/
}  // namespace fsm
}  // namespace hello_world
