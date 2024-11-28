/**
 *******************************************************************************
 * @file      : laser.cpp
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
/* Includes ------------------------------------------------------------------*/
#include "laser.hpp"

/* 开启 TIM 才允许编译 */
#ifdef HAL_TIM_MODULE_ENABLED

#include "assert.hpp"

namespace hello_world
{
namespace laser
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

Laser::Laser(TIM_HandleTypeDef *htim, uint32_t channel, float init_lumin_pct)
    : htim_(htim), channel_(channel)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(htim != nullptr, "Error TIM handle");
  HW_ASSERT(IS_TIM_INSTANCE(htim->Instance), "Error TIM handle");
  HW_ASSERT(IS_TIM_CHANNELS(channel), "Error TIM channel: %d", channel);
#pragma endregion

  __HAL_TIM_SET_COMPARE(htim_, channel_, luminPct2Cmp(init_lumin_pct));
  HAL_TIM_PWM_Start(htim_, channel_);
}

Laser& Laser::operator=(const Laser& other)
{
  if (this != &other) {
    htim_ = other.htim_;
    channel_ = other.channel_;

    __HAL_TIM_SET_COMPARE(htim_, channel_, luminPct2Cmp(0));
    HAL_TIM_PWM_Start(htim_, channel_);
  }
  return *this;
}

Laser::Laser(Laser&& other)
{
  htim_ = other.htim_;
  channel_ = other.channel_;
  other.htim_ = nullptr;

  __HAL_TIM_SET_COMPARE(htim_, channel_, luminPct2Cmp(0));
  HAL_TIM_PWM_Start(htim_, channel_);
}

Laser& Laser::operator=(Laser&& other)
{
  if (this != &other) {
    htim_ = other.htim_;
    channel_ = other.channel_;
    other.htim_ = nullptr;

    __HAL_TIM_SET_COMPARE(htim_, channel_, luminPct2Cmp(0));
    HAL_TIM_PWM_Start(htim_, channel_);
  }
  return *this;
}

void Laser::init(
    TIM_HandleTypeDef* htim, uint32_t channel, float init_lumin_pct)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(htim != nullptr, "Error TIM handle");
  HW_ASSERT(IS_TIM_INSTANCE(htim->Instance), "Error TIM handle");
  HW_ASSERT(IS_TIM_CHANNELS(channel), "Error TIM channel: %d", channel);
#pragma endregion

  htim_ = htim;
  channel_ = channel;

  __HAL_TIM_SET_COMPARE(htim_, channel_, luminPct2Cmp(init_lumin_pct));
  HAL_TIM_PWM_Start(htim_, channel_);
}
/* Private function definitions ----------------------------------------------*/
}  // namespace laser
}  // namespace hello_world

#endif /* HAL_TIM_MODULE_ENABLED */
