/**
 *******************************************************************************
 * @file      : fir.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-12-25      CaiKunzhen      1. 完成初版编写
 *  V1.0.0      2023-12-30      CaiKunzhen      1. 完成初版测试
 *  V1.1.0      2004-07-14      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "fir.hpp"

#include "assert.hpp"

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

Fir::Fir(const float h_arr[], size_t m, size_t dim) : Filter(dim), m_(m)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(dim > 0, "dim must be greater than 0");
  HW_ASSERT(m > 0, "m must be greater than 0");
#pragma endregion

  x_queues_ = new internal::LoopQueue[dim_];
  for (size_t i = 0; i < dim_; i++) {
    x_queues_[i].init(m_ + 1);
  }

  h_arr_ = Allocator<float>::allocate(m_ + 1);
  memcpy(h_arr_, h_arr, sizeof(float) * (m_ + 1));
}

Fir::Fir(const Fir& other) : Filter(other)
{
  m_ = other.m_;
  is_init_ = other.is_init_;
  x_queues_ = new internal::LoopQueue[dim_];
  for (size_t i = 0; i < dim_; i++) {
    x_queues_[i] = other.x_queues_[i];
  }

  h_arr_ = Allocator<float>::allocate(m_ + 1);
  memcpy(h_arr_, other.h_arr_, sizeof(float) * (m_ + 1));
}

Fir& Fir::operator=(const Fir& other)
{
  if (this == &other) {
    return *this;
  }

  Filter::operator=(other);

  if (x_queues_ != nullptr) {
    delete[] x_queues_;
  }

  if (h_arr_ != nullptr) {
    Allocator<float>::deallocate(h_arr_, m_ + 1);
  }

  m_ = other.m_;
  is_init_ = other.is_init_;
  x_queues_ = new internal::LoopQueue[dim_];
  for (size_t i = 0; i < dim_; i++) {
    x_queues_[i] = other.x_queues_[i];
  }

  h_arr_ = Allocator<float>::allocate(m_ + 1);
  memcpy(h_arr_, other.h_arr_, sizeof(float) * (m_ + 1));

  return *this;
}

Fir::Fir(Fir&& other) : Filter(other)
{
  m_ = other.m_;
  is_init_ = other.is_init_;
  x_queues_ = other.x_queues_;
  other.x_queues_ = nullptr;

  h_arr_ = other.h_arr_;
  other.h_arr_ = nullptr;
}

Fir& Fir::operator=(Fir&& other)
{
  if (this == &other) {
    return *this;
  }

  Filter::operator=(other);

  if (x_queues_ != nullptr) {
    delete[] x_queues_;
  }

  if (h_arr_ != nullptr) {
    Allocator<float>::deallocate(h_arr_, m_ + 1);
  }

  m_ = other.m_;
  is_init_ = other.is_init_;
  x_queues_ = other.x_queues_;
  other.x_queues_ = nullptr;

  h_arr_ = other.h_arr_;
  other.h_arr_ = nullptr;

  return *this;
}

Fir::~Fir(void)
{
  delete[] x_queues_;
  Allocator<float>::deallocate(h_arr_, m_ + 1);
}

void Fir::calc(const float in_arr[], float out_arr[])
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(in_arr != nullptr, "in must not be nullptr");
  HW_ASSERT(out_arr != nullptr, "out must not be nullptr");
#pragma endregion

  for (size_t i = 0; i < dim_; i++) {
    out_arr[i] = 0;
    if (!is_init_) {
      x_queues_[i].fill(in_arr[i]);
    }
    x_queues_[i].push(in_arr[i]);
    for (size_t j = 0; j < m_ + 1; j++) {
      out_arr[i] += h_arr_[j] * x_queues_[i].at(m_ - j);
    }
  }

  is_init_ = true;
}

void Fir::reset(void)
{
  for (size_t i = 0; i < dim_; i++) {
    x_queues_[i].reset();
  }

  is_init_ = false;
}

void Fir::init(const float h_arr[], size_t m, size_t dim)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(dim > 0, "dim must be greater than 0");
  HW_ASSERT(m > 0, "m must be greater than 0");
#pragma endregion

  if (x_queues_ != nullptr) {
    delete[] x_queues_;
  }

  if (h_arr_ != nullptr) {
    Allocator<float>::deallocate(h_arr_, m_ + 1);
  }

  dim_ = dim;

  m_ = m;

  x_queues_ = new internal::LoopQueue[dim_];
  for (size_t i = 0; i < dim_; i++) {
    x_queues_[i].init(m_ + 1);
  }

  h_arr_ = Allocator<float>::allocate(m_ + 1);
  memcpy(h_arr_, h_arr, sizeof(float) * (m_ + 1));

  is_init_ = false;
}

void Fir::setInitValues(const float init_values[], size_t idx)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(init_values != nullptr, "init_values must not be nullptr");
  HW_ASSERT(idx < dim_, "idx must be less than %d", dim_);
#pragma endregion

  x_queues_[idx].setInitValues(init_values);
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace filter
}  // namespace hello_world