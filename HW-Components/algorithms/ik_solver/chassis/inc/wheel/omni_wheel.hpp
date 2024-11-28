/**
 *******************************************************************************
 * @file      : omni_wheel.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_OMNI_WHEEL_HPP_
#define HW_COMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_OMNI_WHEEL_HPP_

/* Includes ------------------------------------------------------------------*/
#include "swedish_wheel.hpp"

namespace hello_world
{
namespace chassis_ik_solver
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
HW_OPTIMIZE_O2_START

class OmniWheel : public SwedishWheel
{
 public:
  typedef Wheel::OptMask OptMask;

  explicit OmniWheel(const WheelParams& params) : SwedishWheel(params) {}
  OmniWheel(const OmniWheel&) = default;
  OmniWheel& operator=(const OmniWheel&) = default;
  OmniWheel(OmniWheel&&) = default;
  OmniWheel& operator=(OmniWheel&&) = default;

  virtual ~OmniWheel(void) = default;

  virtual void setGamma(float gamma) override { params_.gamma = 0; }
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace chassis_ik_solver
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_OMNI_WHEEL_HPP_ */
