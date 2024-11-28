/**
 *******************************************************************************
 * @file      : filter_base.hpp
 * @brief     : 滤波器基类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-12-25      CaiKunzhen      1. 完成初版编写
 *  V1.0.0      2023-12-30      CaiKunzhen      1. 完成初版测试
 *  V1.1.0      2004-07-10      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_FILTER_FILTER_BASE_HPP_
#define HW_COMPONENTS_ALGORITHMS_FILTER_FILTER_BASE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstddef>

#include "allocator.hpp"

namespace hello_world
{
namespace filter
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Filter : public MemMgr
{
 public:
  Filter(void) = default;
  /**
   * @brief       构造函数
   * @param        dim: 输入输出维度
   * @retval       None
   * @note        None
   */
  Filter(size_t dim) : dim_(dim){}
  Filter(const Filter&) = default;
  Filter& operator=(const Filter&) = default;
  Filter(Filter&&) = default;
  Filter& operator=(Filter&&) = default;

  virtual ~Filter(void) = default;

  /* 功能性方法 */

  /**
   * @brief       滤波计算
   * @param        in_arr: 输入数据，长度为 dim_
   * @param        out_arr: 输出数据，长度为 dim_
   * @retval       None
   * @note        None
   */
  virtual void calc(const float in_arr[], float out_arr[]) = 0;

  /**
   * @brief       重置
   * @param        None
   * @retval       None
   * @note        None
   */
  virtual void reset(void) = 0;

  /* 数据修改与获取 */

  size_t dim(void) const { return dim_; }

 protected:
  size_t dim_ = 0;  ///< 输入输出维度
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace filter
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_FILTER_FILTER_BASE_HPP_ */
