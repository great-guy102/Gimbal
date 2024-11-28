/**
 *******************************************************************************
 * @file      : loop_queue.hpp
 * @brief     : 定长循环队列
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-12-25      CaiKunzhen      1. 完成初版编写
 *  V1.0.0      2023-12-30      CaiKunzhen      1. 完成初版测试
 *  V1.1.0      2004-07-10      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  该文件为内部文件，用户不应直接调用
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_FILTER_QUEUE_HPP_
#define HW_COMPONENTS_ALGORITHMS_FILTER_QUEUE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstddef>
#include <cstring>

#include "allocator.hpp"
#include "assert.hpp"
#include "system.hpp"

namespace hello_world
{
namespace filter
{
namespace internal
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START

class LoopQueue : public MemMgr
{
 public:
  LoopQueue(void) = default;
  /**
   * @brief       循环队列构造函数
   * @param        len: 队列长度
   * @retval       None
   * @note        None
   */
  explicit LoopQueue(size_t len) : len_(len), start_idx_(0)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(len > 0, "len must be greater than 0");
#pragma endregion

    datas_ = Allocator<float>::allocate(len_);
    memset(datas_, 0, sizeof(float) * len_);
  }
  LoopQueue(const LoopQueue& other)
  {
    len_ = other.len_;
    start_idx_ = other.start_idx_;
    datas_ = Allocator<float>::allocate(len_);
    memcpy(datas_, other.datas_, sizeof(float) * len_);
  }
  LoopQueue& operator=(const LoopQueue& other)
  {
    if (this == &other) {
      return *this;
    }

    if (datas_ != nullptr) {
      Allocator<float>::deallocate(datas_, len_);
    }

    len_ = other.len_;
    start_idx_ = other.start_idx_;
    datas_ = Allocator<float>::allocate(len_);
    memcpy(datas_, other.datas_, sizeof(float) * len_);

    return *this;
  }
  LoopQueue(LoopQueue&& other)
  {
    len_ = other.len_;
    start_idx_ = other.start_idx_;
    datas_ = other.datas_;
    other.datas_ = nullptr;
  }
  LoopQueue& operator=(LoopQueue&& other)
  {
    if (this == &other) {
      return *this;
    }

    if (datas_ != nullptr) {
      Allocator<float>::deallocate(datas_, len_);
    }

    len_ = other.len_;
    start_idx_ = other.start_idx_;
    datas_ = other.datas_;
    other.datas_ = nullptr;

    return *this;
  }

  virtual ~LoopQueue(void)
  {
    Allocator<float>::deallocate(datas_, len_);
  }

  /* 配置方法 */

  /**
   * @brief       初始化队列，使用默认构造函数后请务必调用此函数
   * @param        len: 队列长度
   * @retval       None
   * @note        None
   */
  void init(size_t len)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(len > 0, "len must be greater than 0");
#pragma endregion

    if (datas_ != nullptr) {
      Allocator<float>::deallocate(datas_, len_);
    }

    len_ = len;
    start_idx_ = 0;
    datas_ = Allocator<float>::allocate(len_);
    memset(datas_, 0, sizeof(float) * len_);
  }

  /*   功能性方法 */

  /**
   * @brief       重置队列大小
   * @param        len: 队列长度
   * @retval       None
   * @note        None
   */
  void resize(size_t len)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(len > 0, "len must be greater than 0");
#pragma endregion

    if (datas_ != nullptr) {
      Allocator<float>::deallocate(datas_, len_);
    }

    len_ = len;
    start_idx_ = 0;
    datas_ = Allocator<float>::allocate(len_);
    memset(datas_, 0, sizeof(float) * len_);
  }

  /* 数据修改与获取 */

  /**
   * @brief       入队
   * @param        in: 输入数据
   * @retval       None
   * @note        会覆盖原有数据
   */
  void push(float in)
  {
    datas_[start_idx_] = in;
    start_idx_ = (start_idx_ + 1) % len_;
  }

  /**
   * @brief       重置
   * @retval       None
   * @note        None
   */
  void reset(void)
  {
    start_idx_ = 0;
    memset(datas_, 0, sizeof(float) * len_);
  }

  /**
   * @brief       获取队列中的数据
   * @param        idx: 数据索引
   * @retval       队列中的数据
   * @note        刚入队的数据索引为 len_ - 1
   */
  float at(size_t idx)
  {
/* 变量检查 */
#pragma region
    HW_ASSERT(idx < len_, "idx must be less than %d", len_);
#pragma endregion

    return datas_[(start_idx_ + idx) % len_];
  }

  /**
   * @brief       设置初始值
   * @param        init_values: 输入数据，长度为 len_
   * @retval       None
   * @note        None
   */
  void setInitValues(const float init_values[])
  {
    start_idx_ = 0;
    memcpy(datas_, init_values, sizeof(float) * len_);
  }

  /**
   * @brief       填充队列
   * @param        val: 填充值
   * @retval       None
   * @note        None
   */
  void fill(float val)
  {
    memset(datas_, val, sizeof(float) * len_);
  }

  size_t size(void) const { return len_; }

 private:
  float* datas_ = nullptr;
  size_t len_ = 0;
  size_t start_idx_ = 0;  ///< 队列起始索引
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace internal
}  // namespace filter
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_FILTER_QUEUE_HPP_ */
