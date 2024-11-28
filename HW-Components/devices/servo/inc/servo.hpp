/**
 *******************************************************************************
 * @file      : servo.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_SERVO_SERVO_HPP_
#define HW_COMPONENTS_DEVICES_SERVO_SERVO_HPP_

/* Includes ------------------------------------------------------------------*/
#include "stm32_hal.hpp"

/* 开启 TIM 才允许编译 */
#ifdef HAL_TIM_MODULE_ENABLED

#include "allocator.hpp"
#include "base.hpp"

namespace hello_world
{
namespace servo
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
struct ServoInfo : public MemMgr {
  /** PWM 频率，单位：Hz，一般 50 Hz ~ 300 Hz */
  float operating_freq = 50;
  uint16_t min_pulse_duration = 500;   ///* 最小脉冲时长，单位：us
  uint16_t max_pulse_duration = 2500;  ///* 最大脉冲时长，单位：us
  float angle_range = 180;             ///* 舵机角度范围（从 0 开始），单位：deg
};

class Servo : public MemMgr
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  Servo(void) = default;
  /**
   * @brief       舵机初始化
   * @param        htim: 定时器句柄指针
   * @param        channel: 定时器 PWM对应输出通道
   * @param        servo_info: 舵机参数
   * @param        init_angle: 舵机初始化角度，单位：deg
   * @retval       None
   * @note        使用前请先确保定时器的频率为 1MHz，舵机角度会被自动限制在设定的角度
   *              范围内，初始化后舵机默认处于开启状态
   */
  Servo(TIM_HandleTypeDef *htim, uint32_t channel,
        const ServoInfo &servo_info, float init_angle);
  Servo(const Servo &) = default;
  Servo &operator=(const Servo &other);
  Servo(Servo &&other);
  Servo &operator=(Servo &&other);

  virtual ~Servo(void) {}

  /* 配置方法 */

  /**
   * @brief       舵机初始化, 使用默认构造函数后请务必调用此函数
   * @param        htim: 定时器句柄指针
   * @param        channel: 定时器 PWM 对应输出通道
   * @param        servo_info: 舵机参数
   * @param        init_angle: 舵机初始化角度，单位：deg
   * @retval       None
   * @note        使用前请先确保定时器的频率为 1MHz，舵机角度会被自动限制在设定的角度
   *              范围内，初始化后舵机默认处于开启状态
   */
  void init(TIM_HandleTypeDef *htim, uint32_t channel,
            const ServoInfo &servo_info, float init_angle);

  /* 功能性方法 */

  /**
   * @brief       使能舵机
   * @retval       None
   * @note        None
   */
  void enable(void) const { HAL_TIM_PWM_Start(htim_, channel_); }

  /**
   * @brief       失能舵机
   * @retval       None
   * @note        None
   */
  void disable(void) const { HAL_TIM_PWM_Stop(htim_, channel_); }

  /**
   * @brief       设定舵机角度
   * @param        angle: 舵机角度，单位：deg
   * @retval       None
   * @note        舵机角度会被自动限制在设定的角度范围内
   */
  void setAngle(float angle) const
  {
    __HAL_TIM_SET_COMPARE(htim_, channel_, angle2Cmp(angle));
  }

  /* 数据修改与获取 */

  const ServoInfo &servo_info(void) const { return servo_info_; }

 private:
  /* 功能性方法 */

  /**
   * @brief       舵机角度转定时器 PWM 比较值
   * @param        angle: 舵机角度，单位：deg
   * @retval       定时器 PWM 比较值
   * @note        舵机角度会被自动限制在设定的角度范围内
   */
  uint32_t angle2Cmp(float angle) const
  {
    angle = Bound(angle, 0.0f, servo_info_.angle_range);
    uint32_t cmp = static_cast<uint32_t>(
        angle *
            (servo_info_.max_pulse_duration - servo_info_.min_pulse_duration) /
            servo_info_.angle_range +
        servo_info_.min_pulse_duration);

    return cmp;
  }

  TIM_HandleTypeDef *htim_ = nullptr;
  uint32_t channel_ = TIM_CHANNEL_1;
  ServoInfo servo_info_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace servo
}  // namespace hello_world

#endif /* HAL_TIM_MODULE_ENABLED */

#endif /* HW_COMPONENTS_DEVICES_SERVO_SERVO_HPP_ */
