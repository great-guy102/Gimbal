/**
 *******************************************************************************
 * @file      : ramp.cpp
 * @brief     : 一阶斜坡滤波器，用于限制数据变化率
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2004-07-14      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  设置了周期（period != 0）时，可用于处理周期性数据，如角度，但输出数据的范围不一定
 *  符合要求，若有严格要求则需进一步处理
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ramp.hpp"

#include <cstring>

#include "allocator.hpp"
#include "assert.hpp"
#include "base.hpp"

namespace hello_world
{
namespace filter
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

HW_OPTIMIZE_O2_START

Ramp::Ramp(float max_changing_rate, float samp_period, float period, size_t dim)
    : Filter(dim),
      max_changing_rate_(max_changing_rate),
      samp_period_(samp_period),
      period_(period)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(dim > 0, "dim must be greater than 0");
  HW_ASSERT(max_changing_rate > 0, "max_changing_rate must be greater than 0");
  HW_ASSERT(samp_period > 0, "samp_period must be greater than 0");
#pragma endregion

  x_arr_ = Allocator<float>::allocate(dim_);
  memset(x_arr_, 0, dim_ * sizeof(float));
}

Ramp::Ramp(const Ramp& other) : Filter(other)
{
  max_changing_rate_ = other.max_changing_rate_;
  samp_period_ = other.samp_period_;
  period_ = other.period_;
  is_init_ = other.is_init_;

  x_arr_ = Allocator<float>::allocate(dim_);
  memcpy(x_arr_, other.x_arr_, dim_ * sizeof(float));
}

Ramp& Ramp::operator=(const Ramp& other)
{
  if (this == &other) {
    return *this;
  }

  if (x_arr_ != nullptr) {
    Allocator<float>::deallocate(x_arr_, dim_);
  }

  Filter::operator=(other);

  max_changing_rate_ = other.max_changing_rate_;
  samp_period_ = other.samp_period_;
  period_ = other.period_;
  is_init_ = other.is_init_;

  x_arr_ = Allocator<float>::allocate(dim_);
  memcpy(x_arr_, other.x_arr_, dim_ * sizeof(float));

  return *this;
}

Ramp::Ramp(Ramp&& other) : Filter(other)
{
  max_changing_rate_ = other.max_changing_rate_;
  samp_period_ = other.samp_period_;
  period_ = other.period_;
  is_init_ = other.is_init_;

  x_arr_ = other.x_arr_;
  other.x_arr_ = nullptr;
}

Ramp& Ramp::operator=(Ramp&& other)
{
  if (this == &other) {
    return *this;
  }

  if (x_arr_ != nullptr) {
    Allocator<float>::deallocate(x_arr_, dim_);
  }

  Filter::operator=(other);

  max_changing_rate_ = other.max_changing_rate_;
  samp_period_ = other.samp_period_;
  period_ = other.period_;
  is_init_ = other.is_init_;

  x_arr_ = other.x_arr_;
  other.x_arr_ = nullptr;

  return *this;
}

Ramp::~Ramp(void)
{
  Allocator<float>::deallocate(x_arr_, dim_);
}

void Ramp::calc(const float in_arr[], float out_arr[])
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(in_arr != nullptr, "in must not be nullptr");
  HW_ASSERT(out_arr != nullptr, "out must not be nullptr");
#pragma endregion

  float max_diff = max_changing_rate_ * samp_period_;
  for (size_t i = 0; i < dim_; i++) {
    if (is_init_) {
      x_arr_[i] = in_arr[i];
    }
    float diff = 0;
    if (period_ > 0) {
      diff = PeriodDataSub(in_arr[i], x_arr_[i], period_);
    } else {
      diff = in_arr[i] - x_arr_[i];
    }

    diff = Bound(diff, max_diff, -max_diff);

    x_arr_[i] += diff;
    out_arr[i] = x_arr_[i];
  }

  is_init_ = false;
}

void Ramp::reset(void)
{
  memset(x_arr_, 0, dim_ * sizeof(float));

  is_init_ = false;
}

void Ramp::init(
    float max_changing_rate, float samp_period, float period, size_t dim)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(dim > 0, "dim must be greater than 0");
  HW_ASSERT(max_changing_rate > 0, "max_changing_rate must be greater than 0");
  HW_ASSERT(samp_period > 0, "samp_period must be greater than 0");
#pragma endregion

  if (x_arr_ != nullptr) {
    Allocator<float>::deallocate(x_arr_, dim_);
  }

  dim_ = dim;

  max_changing_rate_ = max_changing_rate;
  samp_period_ = samp_period;
  period_ = period;

  x_arr_ = Allocator<float>::allocate(dim_);
  memset(x_arr_, 0, dim_ * sizeof(float));

  is_init_ = false;
}

void Ramp::setInitValues(const float x_arr[])
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(x_arr != nullptr, "x_arr must not be nullptr");
#pragma endregion

  memcpy(x_arr_, x_arr, dim_ * sizeof(float));
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace filter
}  // namespace hello_world
