/**
 *******************************************************************************
 * @file      : iir.cpp
 * @brief     : IIR 滤波器类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-12-25      CaiKunzhen      1. 完成初版编写
 *  V1.0.0      2023-12-30      CaiKunzhen      1. 完成初版测试
 *  V1.1.0      2004-07-14      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  IIR 可能会出现发散问题，使用时注意检查输出结果是否发散
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "iir.hpp"

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

Iir::Iir(
    const float b_arr[], const float a_arr[], size_t m, size_t n, size_t dim)
    : Filter(dim), m_(m), n_(n)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(dim > 0, "dim must be greater than 0");
  HW_ASSERT(n > 0, "n must be greater than 0");
  HW_ASSERT(m <= n, "m must be less than or equal to n");
#pragma endregion

  x_queues_ = new internal::LoopQueue[dim_];
  y_queues_ = new internal::LoopQueue[dim_];
  for (size_t i = 0; i < dim_; i++) {
    x_queues_[i].init(m_ + 1);
    y_queues_[i].init(n_ + 1);
  }

  b_arr_ = Allocator<float>::allocate(m_ + 1);
  a_arr_ = Allocator<float>::allocate(n_);
  memcpy(b_arr_, b_arr, sizeof(float) * (m_ + 1));
  memcpy(a_arr_, a_arr, sizeof(float) * n_);
}

Iir::Iir(const Iir& other) : Filter(other)
{
  m_ = other.m_;
  n_ = other.n_;
  is_init_ = other.is_init_;
  x_queues_ = new internal::LoopQueue[dim_];
  y_queues_ = new internal::LoopQueue[dim_];
  for (size_t i = 0; i < dim_; i++) {
    x_queues_[i] = other.x_queues_[i];
    y_queues_[i] = other.y_queues_[i];
  }

  b_arr_ = Allocator<float>::allocate(m_ + 1);
  a_arr_ = Allocator<float>::allocate(n_);
  memcpy(b_arr_, other.b_arr_, sizeof(float) * (m_ + 1));
  memcpy(a_arr_, other.a_arr_, sizeof(float) * n_);
}

Iir& Iir::operator=(const Iir& other)
{
  if (this == &other) {
    return *this;
  }

  Filter::operator=(other);

  if (x_queues_ != nullptr) {
    delete[] x_queues_;
  }

  if (y_queues_ != nullptr) {
    delete[] y_queues_;
  }

  if (b_arr_ != nullptr) {
    Allocator<float>::deallocate(b_arr_, m_ + 1);
  }

  if (a_arr_ != nullptr) {
    Allocator<float>::deallocate(a_arr_, n_);
  }

  m_ = other.m_;
  n_ = other.n_;
  is_init_ = other.is_init_;
  x_queues_ = new internal::LoopQueue[dim_];
  y_queues_ = new internal::LoopQueue[dim_];
  for (size_t i = 0; i < dim_; i++) {
    x_queues_[i] = other.x_queues_[i];
    y_queues_[i] = other.y_queues_[i];
  }

  b_arr_ = Allocator<float>::allocate(m_ + 1);
  a_arr_ = Allocator<float>::allocate(n_);
  memcpy(b_arr_, other.b_arr_, sizeof(float) * (m_ + 1));
  memcpy(a_arr_, other.a_arr_, sizeof(float) * n_);

  return *this;
}

Iir::Iir(Iir&& other) : Filter(other)
{
  m_ = other.m_;
  n_ = other.n_;
  is_init_ = other.is_init_;
  x_queues_ = other.x_queues_;
  y_queues_ = other.y_queues_;
  other.x_queues_ = nullptr;
  other.y_queues_ = nullptr;

  b_arr_ = other.b_arr_;
  a_arr_ = other.a_arr_;
  other.b_arr_ = nullptr;
  other.a_arr_ = nullptr;
}

Iir& Iir::operator=(Iir&& other)
{
  if (this == &other) {
    return *this;
  }

  Filter::operator=(other);

  if (x_queues_ != nullptr) {
    delete[] x_queues_;
  }

  if (y_queues_ != nullptr) {
    delete[] y_queues_;
  }

  if (b_arr_ != nullptr) {
    Allocator<float>::deallocate(b_arr_, m_ + 1);
  }

  if (a_arr_ != nullptr) {
    Allocator<float>::deallocate(a_arr_, n_);
  }

  m_ = other.m_;
  n_ = other.n_;
  is_init_ = other.is_init_;
  x_queues_ = other.x_queues_;
  y_queues_ = other.y_queues_;
  other.x_queues_ = nullptr;
  other.y_queues_ = nullptr;

  b_arr_ = other.b_arr_;
  a_arr_ = other.a_arr_;
  other.b_arr_ = nullptr;
  other.a_arr_ = nullptr;

  return *this;
}

Iir::~Iir(void)
{
  delete[] x_queues_;
  delete[] y_queues_;
  Allocator<float>::deallocate(b_arr_, m_ + 1);
  Allocator<float>::deallocate(a_arr_, n_);
}

void Iir::calc(const float in_arr[], float out_arr[])
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(in_arr != nullptr, "in must not be nullptr");
  HW_ASSERT(out_arr != nullptr, "out must not be nullptr");
#pragma endregion

  for (size_t i = 0; i < dim_; i++) {
    float yn = 0;
    if (!is_init_) {
      x_queues_[i].fill(in_arr[i]);
      y_queues_[i].fill(in_arr[i]);
    }
    x_queues_[i].push(in_arr[i]);
    for (size_t j = 0; j < m_ + 1; j++) {
      yn += b_arr_[j] * x_queues_[i].at(m_ - j);
    }
    for (size_t j = 0; j < n_; j++) {
      yn -= a_arr_[j] * y_queues_[i].at(n_ - j);
    }

    y_queues_[i].push(yn);
    out_arr[i] = yn;
  }

  is_init_ = true;
}

void Iir::reset(void)
{
  for (size_t i = 0; i < dim_; i++) {
    x_queues_[i].reset();
    y_queues_[i].reset();
  }

  is_init_ = false;
}

void Iir::init(const float b_arr[], const float a_arr[],
               size_t m, size_t n, size_t dim)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(dim > 0, "dim must be greater than 0");
  HW_ASSERT(n > 0, "n must be greater than 0");
  HW_ASSERT(m <= n, "m must be less than or equal to n");
#pragma endregion

  if (x_queues_ != nullptr) {
    delete[] x_queues_;
  }

  if (y_queues_ != nullptr) {
    delete[] y_queues_;
  }

  if (b_arr_ != nullptr) {
    Allocator<float>::deallocate(b_arr_, m_ + 1);
  }

  if (a_arr_ != nullptr) {
    Allocator<float>::deallocate(a_arr_, n_);
  }

  dim_ = dim;

  m_ = m;
  n_ = n;

  x_queues_ = new internal::LoopQueue[dim_];
  y_queues_ = new internal::LoopQueue[dim_];
  for (size_t i = 0; i < dim_; i++) {
    x_queues_[i].init(m_ + 1);
    y_queues_[i].init(n_ + 1);
  }

  b_arr_ = Allocator<float>::allocate(m_ + 1);
  a_arr_ = Allocator<float>::allocate(n_);
  memcpy(b_arr_, b_arr, sizeof(float) * (m_ + 1));
  memcpy(a_arr_, a_arr, sizeof(float) * n_);

  is_init_ = false;
}

void Iir::setInitValues(const float x_arr[], const float y_arr[], size_t idx)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(idx < dim_, "idx must be less than %d", dim_);
#pragma endregion

  if (x_arr != nullptr) {
    x_queues_[idx].setInitValues(x_arr);
  }

  if (y_arr != nullptr) {
    y_queues_[idx].setInitValues(y_arr);
  }
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace filter
}  // namespace hello_world