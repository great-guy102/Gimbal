/**
 *******************************************************************************
 * @file      : td.cpp
 * @brief     : 一阶微分跟踪器，用于计算微分
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-12-25      CaiKunzhen      1. 完成初版编写
 *  V1.0.0      2023-12-30      CaiKunzhen      1. 完成初版测试
 *  V1.1.0      2004-07-14      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  1. 设置了周期（period != 0）时，可用于处理周期性数据，如角度
 *  2. 截止频率 r 不能过小，同时不能超过采样频率，否则极容易导致发散
 *  3. 该滤波器在某些情况下可能会发散，使用是建议通过 is_divergence 方法检查
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "td.hpp"

#include <cstring>

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

Td::Td(float r, float samp_period, float period, size_t dim)
    : Filter(dim), r_(r), samp_period_(samp_period), period_(period)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(dim > 0, "dim must be greater than 0");
  HW_ASSERT(r > 0, "r must be greater than 0");
  HW_ASSERT(samp_period > 0, "samp_period must be greater than 0");
#pragma endregion

  x_arr_ = Allocator<float>::allocate(dim_);
  dx_arr_ = Allocator<float>::allocate(dim_);

  memset(x_arr_, 0, dim_ * sizeof(float));
  memset(dx_arr_, 0, dim_ * sizeof(float));
}

Td::Td(const Td& other) : Filter(other)
{
  r_ = other.r_;
  samp_period_ = other.samp_period_;
  period_ = other.period_;
  is_init_ = other.is_init_;
  is_divergence_ = other.is_divergence_;

  x_arr_ = Allocator<float>::allocate(dim_);
  dx_arr_ = Allocator<float>::allocate(dim_);
  memcpy(x_arr_, other.x_arr_, dim_ * sizeof(float));
  memcpy(dx_arr_, other.dx_arr_, dim_ * sizeof(float));
}

Td& Td::operator=(const Td& other)
{
  if (this == &other) {
    return *this;
  }

  if (x_arr_ != nullptr) {
    Allocator<float>::deallocate(x_arr_, dim_);
  }

  if (dx_arr_ != nullptr) {
    Allocator<float>::deallocate(dx_arr_, dim_);
  }

  Filter::operator=(other);

  r_ = other.r_;
  samp_period_ = other.samp_period_;
  period_ = other.period_;
  is_init_ = other.is_init_;
  is_divergence_ = other.is_divergence_;

  x_arr_ = Allocator<float>::allocate(dim_);
  dx_arr_ = Allocator<float>::allocate(dim_);
  memcpy(x_arr_, other.x_arr_, dim_ * sizeof(float));
  memcpy(dx_arr_, other.dx_arr_, dim_ * sizeof(float));

  return *this;
}

Td::Td(Td&& other) : Filter(other)
{
  r_ = other.r_;
  samp_period_ = other.samp_period_;
  period_ = other.period_;
  is_init_ = other.is_init_;
  is_divergence_ = other.is_divergence_;

  x_arr_ = other.x_arr_;
  dx_arr_ = other.dx_arr_;
  other.x_arr_ = nullptr;
  other.dx_arr_ = nullptr;
}

Td& Td::operator=(Td&& other)
{
  if (this == &other) {
    return *this;
  }

  if (x_arr_ != nullptr) {
    Allocator<float>::deallocate(x_arr_, dim_);
  }

  if (dx_arr_ != nullptr) {
    Allocator<float>::deallocate(dx_arr_, dim_);
  }

  Filter::operator=(other);

  r_ = other.r_;
  samp_period_ = other.samp_period_;
  period_ = other.period_;
  is_init_ = other.is_init_;
  is_divergence_ = other.is_divergence_;

  x_arr_ = other.x_arr_;
  dx_arr_ = other.dx_arr_;
  other.x_arr_ = nullptr;
  other.dx_arr_ = nullptr;

  return *this;
}

Td::~Td(void)
{
  Allocator<float>::deallocate(x_arr_, dim_);
  Allocator<float>::deallocate(dx_arr_, dim_);
}

void Td::calc(const float in_ls[], float out_ls[])
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(in_ls != nullptr, "in must not be nullptr");
  HW_ASSERT(out_ls != nullptr, "out must not be nullptr");
#pragma endregion

  float diff_x = 0;
  for (size_t i = 0; i < dim_; i++) {
    if (!is_init_) {
      x_arr_[i] = in_ls[i];
    }

    if (period_ > 0) {
      diff_x = PeriodDataSub(x_arr_[i], in_ls[i], period_);
      x_arr_[i] = NormPeriodData(
          0, period_, x_arr_[i] + samp_period_ * dx_arr_[i]);
    } else {
      diff_x = x_arr_[i] - in_ls[i];
      x_arr_[i] = x_arr_[i] + samp_period_ * dx_arr_[i];
    }

    dx_arr_[i] =
        dx_arr_[i] -
        samp_period_ * (r_ * r_ * diff_x + 2 * r_ * dx_arr_[i]);

    if (isnanf(dx_arr_[i]) || isinff(dx_arr_[i]) ||
        isnanf(x_arr_[i]) || isinff(x_arr_[i])) {
      is_divergence_ = true;
    }
  }

  is_init_ = true;

  memcpy(out_ls, dx_arr_, dim_ * sizeof(float));
}

void Td::reset(void)
{
  is_init_ = false;
  is_divergence_ = false;
  memset(x_arr_, 0, dim_ * sizeof(float));
  memset(dx_arr_, 0, dim_ * sizeof(float));
}

void Td::init(float r, float samp_period, float period, size_t dim)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(dim > 0, "dim must be greater than 0");
  HW_ASSERT(r > 0, "r must be greater than 0");
  HW_ASSERT(samp_period > 0, "samp_period must be greater than 0");
#pragma endregion

  if (x_arr_ != nullptr) {
    Allocator<float>::deallocate(x_arr_, dim_);
  }

  if (dx_arr_ != nullptr) {
    Allocator<float>::deallocate(dx_arr_, dim_);
  }

  dim_ = dim;

  r_ = r;
  samp_period_ = samp_period;
  period_ = period;

  x_arr_ = Allocator<float>::allocate(dim_);
  dx_arr_ = Allocator<float>::allocate(dim_);

  memset(x_arr_, 0, dim_ * sizeof(float));
  memset(dx_arr_, 0, dim_ * sizeof(float));

  is_init_ = false;
  is_divergence_ = false;
}

void Td::setInitValues(const float x_ls[], const float dx_ls[])
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(x_ls != nullptr, "x must not be nullptr");
  HW_ASSERT(dx_ls != nullptr, "dx must not be nullptr");
#pragma endregion

  is_init_ = true;
  is_divergence_ = false;
  memcpy(x_arr_, x_ls, dim_ * sizeof(float));
  memcpy(dx_arr_, dx_ls, dim_ * sizeof(float));
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace filter
}  // namespace hello_world