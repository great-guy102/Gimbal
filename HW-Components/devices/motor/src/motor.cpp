/**
 *******************************************************************************
 * @file      : motor.cpp
 * @brief     : 电机汇总
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2023-12-04      Caikunzhen      1. 完成测试
 *  V1.0.1      2023-12-14      Caikunzhen      1. 添加电机简单工厂函数
 *  V1.1.0      2024-07-11      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "motor.hpp"

#include "assert.hpp"

namespace hello_world
{
namespace motor
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

HW_OPTIMIZE_O2_START

Motor* CreateMotor(MotorType motor_type, uint8_t id,
                   const OptionalParams& opt)
{
  switch (motor_type) {
    case MotorType::kGM6020:
      return new GM6020(id, opt);
    case MotorType::kM2006:
      return new M2006(id, opt);
    case MotorType::kM3508:
      return new M3508(id, opt);
    case MotorType::kM8910:
      return new M8910(id, opt);
    case MotorType::kA1:
      return new A1(id, opt);
    case MotorType::kDM_J4310:
      return new DM_J4310(id, opt);
    case MotorType::kDM_J8006:
      return new DM_J8006(id, opt);
    case MotorType::kMF9025v2:
      return new MF9025v2(id, opt);
    case MotorType::kMG6012Ei36:
      return new MG6012Ei36(id, opt);
    case MotorType::kMG8016E:
      return new MG8016E(id, opt);
    case MotorType::kGO_M8010:
      return new GO_M8010(id, opt);
    default:
      HW_ASSERT(0, "Error motor type");
      return nullptr;
  }
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace motor
}  // namespace hello_world
