/**
 *******************************************************************************
 * @file      : wheel.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "assert.hpp"
#include "wheel/wheel.hpp"

namespace hello_world
{
namespace chassis_ik_solver
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

HW_OPTIMIZE_O2_START

Wheel* CreateWheel(WheelType wheel_type, const WheelParams& params)
{
  switch (wheel_type) {
    case WheelType::kFixedStandard:
      return new FixedStandardWheel(params);
    case WheelType::kSteeredStandard:
      return new SteeredStandardWheel(params);
    case WheelType::kSwedish:
      return new SwedishWheel(params);
    case WheelType::kMecanum:
      return new MecanumWheel(params);
    case WheelType::kOmni:
      return new OmniWheel(params);
    case WheelType::kSpherical:
      return new SphericalWheel(params);
    default:
      HW_ASSERT(0, "Illegal wheel type %d", wheel_type);
      return nullptr;
  }
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace chassis_ik_solver
}  // namespace hello_world
