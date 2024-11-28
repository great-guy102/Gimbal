/**
 *******************************************************************************
 * @file      : observer_base.cpp
 * @brief     : 观测器基类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-12-27      Caikunzhen      1. 完成初版编写（未测试）
 *  V1.0.0      2004-07-13      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "observer_base.hpp"

#include "assert.hpp"

namespace hello_world
{
namespace observer
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

HW_OPTIMIZE_O2_START

Observer::Observer(size_t x_dim, size_t z_dim, size_t u_dim)
    : x_dim_(x_dim), z_dim_(z_dim), u_dim_(u_dim)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(x_dim > 0, "x_dim must be greater than 0");
  HW_ASSERT(z_dim > 0, "z_dim must be greater than 0");
  HW_ASSERT(u_dim > 0, "u_dim must be greater than 0");
#pragma endregion

  x_hat_ = Allocator<float>::allocate(x_dim_);
  memset(x_hat_, 0, x_dim_ * sizeof(float));
}

Observer::Observer(const Observer& other)
    : x_dim_(other.x_dim_), z_dim_(other.z_dim_), u_dim_(other.u_dim_)
{
  x_hat_ = Allocator<float>::allocate(x_dim_);
  memcpy(x_hat_, other.x_hat_, x_dim_ * sizeof(float));
}

Observer& Observer::operator=(const Observer& other)
{
  if (this != &other) {
    if (x_hat_ != nullptr) {
      Allocator<float>::deallocate(x_hat_, x_dim_);
    }

    x_dim_ = other.x_dim_;
    z_dim_ = other.z_dim_;
    u_dim_ = other.u_dim_;

    x_hat_ = Allocator<float>::allocate(x_dim_);
    memcpy(x_hat_, other.x_hat_, x_dim_ * sizeof(float));
  }

  return *this;
}

Observer::Observer(Observer&& other)
    : x_dim_(other.x_dim_), z_dim_(other.z_dim_), u_dim_(other.u_dim_)
{
  x_hat_ = other.x_hat_;
  other.x_hat_ = nullptr;
}

Observer& Observer::operator=(Observer&& other)
{
  if (this != &other) {
    if (x_hat_ != nullptr) {
      Allocator<float>::deallocate(x_hat_, x_dim_);
    }

    x_dim_ = other.x_dim_;
    z_dim_ = other.z_dim_;
    u_dim_ = other.u_dim_;

    x_hat_ = other.x_hat_;
    other.x_hat_ = nullptr;
  }

  return *this;
}

Observer::~Observer(void)
{
  Allocator<float>::deallocate(x_hat_, x_dim_);
}

void Observer::getX(float x_hat[]) const
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(x_hat != nullptr, "x_hat must not be nullptr");
#pragma endregion

  memcpy(x_hat, x_hat_, x_dim_ * sizeof(float));
}

void Observer::init(size_t x_dim, size_t z_dim, size_t u_dim)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(x_dim > 0, "x_dim must be greater than 0");
  HW_ASSERT(z_dim > 0, "z_dim must be greater than 0");
  HW_ASSERT(u_dim > 0, "u_dim must be greater than 0");
#pragma endregion

  if (x_hat_ != nullptr) {
    Allocator<float>::deallocate(x_hat_, x_dim_);
  }

  x_dim_ = x_dim;
  z_dim_ = z_dim;
  u_dim_ = u_dim;

  x_hat_ = Allocator<float>::allocate(x_dim_);
  memset(x_hat_, 0, x_dim_ * sizeof(float));
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace observer
}  // namespace hello_world