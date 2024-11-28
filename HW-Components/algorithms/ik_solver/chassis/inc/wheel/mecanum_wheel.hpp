/**
 *******************************************************************************
 * @file      : mecanum_wheel.hpp
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
#ifndef HW_COMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_MECANUM_WHEEL_HPP_
#define HW_COMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_MECANUM_WHEEL_HPP_

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

class MecanumWheel : public SwedishWheel
{
 public:
  typedef Wheel::OptMask OptMask;

  explicit MecanumWheel(const WheelParams& params) : SwedishWheel(params) {}
  MecanumWheel(const MecanumWheel&) = default;
  MecanumWheel& operator=(const MecanumWheel&) = default;
  MecanumWheel(MecanumWheel&&) = default;
  MecanumWheel& operator=(MecanumWheel&&) = default;

  virtual ~MecanumWheel(void) = default;

  virtual void setGamma(float gamma) override { params_.gamma = M_PI_4; }
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace chassis_ik_solver
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_MECANUM_WHEEL_HPP_ */
