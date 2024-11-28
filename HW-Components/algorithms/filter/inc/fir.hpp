/**
 *******************************************************************************
 * @file      : fir.hpp
 * @brief     : FIR 滤波器类
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_FILTER_FIR_HPP_
#define HW_COMPONENTS_ALGORITHMS_FILTER_FIR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "filter_base.hpp"
#include "loop_queue.hpp"
#include "system.hpp"

namespace hello_world
{
namespace filter
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START

class Fir : public Filter
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  Fir(void) = default;
  /**
   * @brief       FIR滤波器初始化
   * @param        h_arr: 前向系数，[h0, h1, h2, ..., hm], 注意大小为 m+1
   * @param        m: 前向系数阶数（>0）
   * @param        dim: 输入输出维度
   * @retval       None
   * @note        H(z) = h0 + h1*z^(-1) + h2*z^(-2) + ... + hm*z^(-m)
   */
  Fir(const float h_arr[], size_t m, size_t dim = 1);
  Fir(const Fir& other);
  Fir& operator=(const Fir& other);
  Fir(Fir&& other);
  Fir& operator=(Fir&& other);

  virtual ~Fir(void);

  /* 重载方法 */

  /**
   * @brief       计算输入数据的滤波结果
   * @param        in_arr: 输入数据，大小为 dim_
   * @param        out_arr: 输出数据，大小为 dim_
   * @note        初次调用时，输入数据为初始值
   */
  virtual void calc(const float in_arr[], float out_arr[]) override;

  /**
   * @brief       重置
   * @param        None
   * @retval       None
   * @note        None
   */
  virtual void reset(void) override;

  /* 配置方法 */

  /**
   * @brief       FIR滤波器初始化，使用默认构造函数后请务必调用此函数
   * @param        h_arr: 前向系数，[h0, h1, h2, ..., hm], 注意大小为 m+1
   * @param        m: 前向系数阶数（>0）
   * @param        dim: 输入输出维度
   * @retval       None
   * @note        H(z) = h0 + h1*z^(-1) + h2*z^(-2) + ... + hm*z^(-m)
   */
  void init(const float h_arr[], size_t m, size_t dim = 1);

  /**
   * @brief       设置初始值
   * @param        init_values: 初始值，大小为 m+1
   * @param        idx: 输入数据索引，[0, dim_)
   * @retval       None
   * @note        None
   */
  void setInitValues(const float init_values[], size_t idx);

 private:
  size_t m_ = 0;  ///< 前向系数阶数
  internal::LoopQueue* x_queues_ = nullptr;
  float* h_arr_ = nullptr;
  bool is_init_ = false; ///< x_queues_ 是否赋了初值
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace filter
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_FILTER_FIR_HPP_ */
