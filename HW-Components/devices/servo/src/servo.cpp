/**
 *******************************************************************************
 * @file      : servo.cpp
 * @brief     : 舵机类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-27      Caikunzhen      1. 未测试版本
 *  V1.0.0      2023-12-15      Caikunzhen      1. 完成测试
 *  V1.1.0      2024-07-13      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  1. 使用前请先确保定时器的频率为 1MHz，同时 STM32CubeMX 配置文件中 TIM 的 Counter
 *  Period 需大于 0，否则会报错
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
#include "servo.hpp"

/* 开启 TIM 才允许编译 */
#ifdef HAL_TIM_MODULE_ENABLED

#include "assert.hpp"

namespace hello_world
{
namespace servo
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
static const float kTimerFreq = 1e6f;
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

Servo::Servo(TIM_HandleTypeDef* htim, uint32_t channel,
             const ServoInfo& servo_info, float init_angle)
    : htim_(htim), channel_(channel), servo_info_(servo_info)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(htim != nullptr, "Error TIM handle");
  HW_ASSERT(IS_TIM_INSTANCE(htim->Instance), "Error TIM handle");
  HW_ASSERT(IS_TIM_CHANNELS(channel), "Error TIM channel: %d", channel);
  HW_ASSERT(servo_info.operating_freq > 0, "Error Operating freqency: %.1f",
            servo_info.operating_freq);
  HW_ASSERT(servo_info.min_pulse_duration < servo_info.max_pulse_duration &&
                servo_info.min_pulse_duration > 0u &&
                servo_info.max_pulse_duration <
                    static_cast<uint16_t>(1e6f / servo_info.operating_freq),
            "Error pulse duration");
  HW_ASSERT(servo_info.angle_range <= 360.0f,
            "Error angle range: %d", servo_info.angle_range);
#pragma endregion

  uint32_t auto_reload =
      static_cast<uint32_t>(kTimerFreq / servo_info_.operating_freq) - 1;
  if (auto_reload == 0) {  // 防止 auto_reload 为 0
    auto_reload = 1;
  }
  __HAL_TIM_SET_AUTORELOAD(htim_, auto_reload);
  __HAL_TIM_SET_COMPARE(htim_, channel_, angle2Cmp(init_angle));
  HAL_TIM_PWM_Start(htim_, channel_);
}

Servo& Servo::operator=(const Servo& other)
{
  if (this != &other) {
    htim_ = other.htim_;
    channel_ = other.channel_;
    servo_info_ = other.servo_info_;

    uint32_t auto_reload =
        static_cast<uint32_t>(kTimerFreq / servo_info_.operating_freq) - 1;
    if (auto_reload == 0) {  // 防止 auto_reload 为 0
      auto_reload = 1;
    }
    __HAL_TIM_SET_AUTORELOAD(htim_, auto_reload);
    __HAL_TIM_SET_COMPARE(htim_, channel_, angle2Cmp(0));
    HAL_TIM_PWM_Start(htim_, channel_);
  }
  return *this;
}

Servo::Servo(Servo&& other)
{
  htim_ = other.htim_;
  channel_ = other.channel_;
  servo_info_ = other.servo_info_;

  other.htim_ = nullptr;

  uint32_t auto_reload =
      static_cast<uint32_t>(kTimerFreq / servo_info_.operating_freq) - 1;
  if (auto_reload == 0) {  // 防止 auto_reload 为 0
    auto_reload = 1;
  }
  __HAL_TIM_SET_AUTORELOAD(htim_, auto_reload);
  __HAL_TIM_SET_COMPARE(htim_, channel_, angle2Cmp(0));
  HAL_TIM_PWM_Start(htim_, channel_);
}

Servo& Servo::operator=(Servo&& other)
{
  if (this != &other) {
    htim_ = other.htim_;
    channel_ = other.channel_;
    servo_info_ = other.servo_info_;

    other.htim_ = nullptr;

    uint32_t auto_reload =
        static_cast<uint32_t>(kTimerFreq / servo_info_.operating_freq) - 1;
    if (auto_reload == 0) {  // 防止 auto_reload 为 0
      auto_reload = 1;
    }
    __HAL_TIM_SET_AUTORELOAD(htim_, auto_reload);
    __HAL_TIM_SET_COMPARE(htim_, channel_, angle2Cmp(0));
    HAL_TIM_PWM_Start(htim_, channel_);
  }
  return *this;
}

void Servo::init(TIM_HandleTypeDef* htim, uint32_t channel,
                 const ServoInfo& servo_info, float init_angle)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(htim != nullptr, "Error TIM handle");
  HW_ASSERT(IS_TIM_INSTANCE(htim->Instance), "Error TIM handle");
  HW_ASSERT(IS_TIM_CHANNELS(channel), "Error TIM channel: %d", channel);
  HW_ASSERT(servo_info.operating_freq > 0, "Error Operating freqency: %.1f",
            servo_info.operating_freq);
  HW_ASSERT(servo_info.min_pulse_duration < servo_info.max_pulse_duration &&
                servo_info.min_pulse_duration > 0u &&
                servo_info.max_pulse_duration <
                    static_cast<uint16_t>(1e6f / servo_info.operating_freq),
            "Error pulse duration");
  HW_ASSERT(servo_info.angle_range <= 360.0f,
            "Error angle range: %d", servo_info.angle_range);
#pragma endregion

  htim_ = htim;
  channel_ = channel;
  servo_info_ = servo_info;

  uint32_t auto_reload =
      static_cast<uint32_t>(kTimerFreq / servo_info_.operating_freq) - 1;
  if (auto_reload == 0) {  // 防止 auto_reload 为 0
    auto_reload = 1;
  }
  __HAL_TIM_SET_AUTORELOAD(htim_, auto_reload);
  __HAL_TIM_SET_COMPARE(htim_, channel_, angle2Cmp(init_angle));
  HAL_TIM_PWM_Start(htim_, channel_);
}
/* Private function definitions ----------------------------------------------*/
}  // namespace servo
}  // namespace hello_world

#endif /* HAL_TIM_MODULE_ENABLED */
