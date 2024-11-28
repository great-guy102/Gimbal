/**
 *******************************************************************************
 * @file      : iir.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_FILTER_IIR_HPP_
#define HW_COMPONENTS_ALGORITHMS_FILTER_IIR_HPP_

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

class Iir : public Filter
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  Iir(void) = default;
  /**
   * @brief       IIR滤波器初始化
   * @param        b_arr: 前向系数，[b0, b1, b2, ..., bm], 注意大小为 m+1
   * @param        a_arr: 反馈系数，[a1, a2, ..., an], 注意大小为 n
   * @param        m: 前向系数阶数（>=0）
   * @param        n: 反馈系数阶数（>0）
   * @param        dim: 输入输出维度
   * @retval       None
   * @note        注意 m<=n，
   *                       b0 + b1*z^(-1) + b2*z^(-2) + ... + bm*z^(-m)
   *              H(z) = ----------------------------------------------
   *                       1 + a1*z^(-1) + a2*z^(-2) + ... + an*z^(-n)
   */
  Iir(const float b_arr[], const float a_arr[],
      size_t m, size_t n, size_t dim = 1);
  Iir(const Iir& other);
  Iir& operator=(const Iir& other);
  Iir(Iir&& other);
  Iir& operator=(Iir&& other);

  virtual ~Iir(void);

  /* 重载方法 */

  /**
   * @brief       计算输入数据的滤波结果
   * @param        in_arr: 输入数据，大小为 dim_
   * @param        out_arr: 输出数据，大小为 dim_
   * @note        None
   */
  virtual void calc(const float in_ls[], float out_ls[]) override;

  /**
   * @brief       重置
   * @param        None
   * @retval       None
   * @note        None
   */
  virtual void reset(void) override;

  /* 配置方法 */

  /**
   * @brief       IIR滤波器初始化，使用默认构造函数后请务必调用此函数
   * @param        b_arr: 前向系数，[b0, b1, b2, ..., bm], 注意大小为 m+1
   * @param        a_arr: 反馈系数，[a1, a2, ..., an], 注意大小为 n
   * @param        m: 前向系数阶数（>=0）
   * @param        n: 反馈系数阶数（>0）
   * @param        dim: 输入输出维度
   * @retval       None
   * @note        注意 m<=n，
   *                       b0 + b1*z^(-1) + b2*z^(-2) + ... + bm*z^(-m)
   *              H(z) = ----------------------------------------------
   *                       1 + a1*z^(-1) + a2*z^(-2) + ... + an*z^(-n)
   */
  void init(const float b_arr[], const float a_arr[],
            size_t m, size_t n, size_t dim = 1);

  /**
   * @brief       设置初始值
   * @param        x_arr: 输入数据，大小为 m+1，为 nullptr 时不设置
   * @param        y_arr: 输出数据，大小为 n+1，为 nullptr 时不设置
   * @param        idx: 输入数据索引，[0, dim_)
   * @retval       None
   * @note        None
   */
  void setInitValues(const float x_arr[], const float y_arr[], size_t idx);

 private:
  size_t m_ = 0;  ///< 前向系数阶数
  size_t n_ = 0;  ///< 反馈系数阶数

  internal::LoopQueue* x_queues_ = nullptr;
  internal::LoopQueue* y_queues_ = nullptr;
  float* b_arr_ = nullptr;
  float* a_arr_ = nullptr;

  bool is_init_ = false;  ///< x_queues_ 与 y_queues_ 是否赋了初值
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace filter
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_FILTER_IIR_HPP_ */
