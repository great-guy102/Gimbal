/**
 *******************************************************************************
 * @file      : laser.hpp
 * @brief     : 红点激光器类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-27      Caikunzhen      1. 未测试版本
 *  V1.0.0      2023-12-05      Caikunzhen      1. 完成测试
 *  V1.1.0      2024-07-13      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  1. 定时器的周期不可太大，否者将引起红点激光的闪烁
 *  2. 由于内部使用了硬件句柄，因此如果计划将实例作为全局变量时（全局变量初始化时对应的
 *  硬件句柄可能会还未初始化完毕），建议采取一下方法：
 *    1）声明指针，后续通过 `new` 的方式进行初始化
 *    2）声明指针，后续通过返回函数（CreateXXXIns）中的静态变量（因为该变量只有在第一
 *    次调用该函数时才会运行初始化程序）进行初始化
 *    3）使用无参构造函数，后续调用 `init` 方法进行初始化
 *    4）使用无参构造函数，后续使用拷贝赋值函数或是移动赋值函数进行初始化
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_LASER_LASER_HPP_
#define HW_COMPONENTS_DEVICES_LASER_LASER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "stm32_hal.hpp"

/* 开启 TIM 才允许编译 */
#ifdef HAL_TIM_MODULE_ENABLED

#include "allocator.hpp"
#include "base.hpp"

namespace hello_world
{
namespace laser
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Laser : public MemMgr
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  Laser(void) = default;
  /**
   * @brief       红点激光器初始化
   * @param        htim: 定时器句柄指针
   * @param        channel: 定时器PWM对应输出通道，可选值为：
   *   @arg        TIM_CHANNEL_x: x=1, 2, 3, 4
   * @param        init_lumin_pct: 初始化百分比亮度（0 ~ 100）
   * @retval       None
   * @note        百分比亮度会被自动限制在合理范围内，初始化后红点激光器默认处于开启状
   *              态
   */
  Laser(TIM_HandleTypeDef *htim, uint32_t channel, float init_lumin_pct);
  Laser(const Laser &) = default;
  Laser &operator=(const Laser &other);
  Laser(Laser &&other);
  Laser &operator=(Laser &&other);

  virtual ~Laser(void) {}

  /* 配置方法 */

  /**
   * @brief       红点激光器初始化，使用默认构造函数后请务必调用此函数
   * @param        htim: 定时器句柄指针
   * @param        channel: 定时器PWM对应输出通道，可选值为：
   *   @arg        TIM_CHANNEL_x: x=1, 2, 3, 4
   * @param        init_lumin_pct: 初始化百分比亮度（0 ~ 100）
   * @retval       None
   * @note        百分比亮度会被自动限制在合理范围内，初始化后红点激光器默认处于开启状
   *              态
   */
  void init(TIM_HandleTypeDef *htim, uint32_t channel, float init_lumin_pct);

  /* 功能性方法 */

  /**
   * @brief       启用红点激光器
   * @retval       None
   * @note        None
   */
  void enable(void) const { HAL_TIM_PWM_Start(htim_, channel_); }

  /**
   * @brief       关闭红点激光器
   * @retval       None
   * @note        None
   */
  void disable(void) const { HAL_TIM_PWM_Stop(htim_, channel_); }

  /* 数据修改与获取 */

  /**
   * @brief       设置红点激光器百分比亮度
   * @param        lumin_pct: 百分比亮度（0 ~ 100）
   * @retval       None
   * @note        百分比亮度会被自动限制在合理范围内
   */
  void setLuminPct(float lumin_pct) const
  {
    __HAL_TIM_SET_COMPARE(htim_, channel_, luminPct2Cmp(lumin_pct));
  }

 private:
  /* 功能性方法 */

  /**
   * @brief       百分比亮度转定时器PWM比较值
   * @param        lumin_pct: 百分比亮度（0 ~ 100）
   * @retval       定时器PWM比较值
   * @note        百分比亮度会被自动限制在合理范围内
   */
  uint32_t luminPct2Cmp(float lumin_pct) const
  {
    lumin_pct = Bound(lumin_pct, 0.0f, 100.0f);
    return static_cast<uint32_t>(
        lumin_pct * __HAL_TIM_GET_AUTORELOAD(htim_) / 100);
  }

  TIM_HandleTypeDef *htim_ = nullptr;
  uint32_t channel_ = TIM_CHANNEL_1;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace laser
}  // namespace hello_world

#endif /* HAL_TIM_MODULE_ENABLED */

#endif /* HW_COMPONENTS_DEVICES_LASER_LASER_HPP_ */
