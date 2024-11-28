/**
 *******************************************************************************
 * @file      : ramp.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_FILTER_RAMP_HPP_
#define HW_COMPONENTS_ALGORITHMS_FILTER_RAMP_HPP_

/* Includes ------------------------------------------------------------------*/
#include <limits>

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

class Ramp : public Filter
{
 public:
  Ramp(void) = default;
  /**
   * @brief       构造函数
   * @param        max_changing_rate: 最大变化率，单位：s^-1
   * @param        samp_period: 采样周期，单位：s
   * @param        period: 数据周期，大于 0 时表示周期性数据
   * @param        dim: 输入输出维度
   * @retval       None
   * @note        None
   */
  Ramp(float max_changing_rate, float samp_period,
       float period = 0.0f, size_t dim = 1);
  Ramp(const Ramp& other);
  Ramp& operator=(const Ramp& other);
  Ramp(Ramp&& other);
  Ramp& operator=(Ramp&& other);

  virtual ~Ramp(void);

  /* 重载方法 */

  /**
   * @brief       滤波计算
   * @param        in_arr: 输入数据，大小为 dim_
   * @param        out_arr: 输出数据，大小为 dim_
   * @retval       None
   * @note        None
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
   * @brief       初始化，使用默认构造函数后请务必调用此函数
   * @param        max_changing_rate: 最大变化率，单位：s^-1
   * @param        samp_period: 采样周期，单位：s
   * @param        period: 数据周期，大于 0 时表示周期性数据
   * @param        dim: 输入输出维度
   * @retval       None
   * @note        None
   */
  void init(float max_changing_rate, float samp_freq,
            float period = 0.0f, size_t dim = 1);

  /**
   * @brief       设置初始值
   * @param        x_arr: 初始值，大小为 dim_
   * @retval       None
   * @note        None
   */
  void setInitValues(const float x_arr[]);

 private:
  /** 最大变化率，单位：s^-1 */
  float max_changing_rate_ = std::numeric_limits<float>::max();
  float samp_period_ = 0.001f;  ///< 采样周期，单位：s
  float period_ = 0;            ///< 数据周期，大于 0 时表示周期性数据

  float* x_arr_ = nullptr;

  bool is_init_ = false;  ///< x_arr_ 是否赋了初值
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace filter
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_FILTER_RAMP_HPP_ */
