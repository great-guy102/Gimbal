/**
 *******************************************************************************
 * @file      : td.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_FILTER_TD_HPP_
#define HW_COMPONENTS_ALGORITHMS_FILTER_TD_HPP_

/* Includes ------------------------------------------------------------------*/
#include "filter_base.hpp"
#include "system.hpp"

namespace hello_world
{
namespace filter
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START

class Td : public Filter
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  Td(void) = default;
  /**
   * @brief       微分跟踪器初始化
   * @param        r: 截止频率，单位：Hz
   * @param        samp_period: 采样周期，单位：s
   * @param        period: 数据周期，大于 0 时表示周期性数据
   * @param        dim: 输入输出维度
   * @retval       None
   * @note        None
   */
  Td(float r, float samp_period, float period = 0.0f, size_t dim = 1);
  Td(const Td& other);
  Td& operator=(const Td& other);
  Td(Td&& other);
  Td& operator=(Td&& other);

  virtual ~Td(void);

  /* 重载方法 */

  /**
   * @brief       计算输入数据的微分
   * @param        in_ls: 输入数据
   * @param        out_ls: 输出数据
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
   * @brief       微分跟踪器初始化
   * @param        r: 截止频率，单位：Hz
   * @param        samp_period: 采样周期，单位：s
   * @param        period: 数据周期，大于 0 时表示周期性数据
   * @param        dim: 输入输出维度
   * @retval       None
   * @note        None
   */
  void init(float r, float samp_period, float period = 0.0f, size_t dim = 1);

  /**
   * @brief       设置初始值
   * @param        x_ls: 初始值，大小为 dim_
   * @param        dx_ls: 初始微分值，大小为 dim_
   * @retval       None
   * @note        None
   */
  void setInitValues(const float x_ls[], const float dx_ls[]);

  /* 数据修改与获取 */

  float period(void) const { return period_; }

  bool is_divergence(void) const { return is_divergence_; }

 private:
  float r_ = 0;                 ///< 截止频率，单位：Hz
  float samp_period_ = 0.001f;  ///< 采样周期，单位：s
  float period_ = 0;            ///< 数据周期，大于 0 时表示周期性数据

  float* x_arr_ = nullptr;
  float* dx_arr_ = nullptr;

  bool is_init_ = false;  ///< x_arr_ 和 dx_arr_ 是否赋了初值
  bool is_divergence_ = false;
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace filter
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_FILTER_TD_HPP_ */
