/**
 *******************************************************************************
 * @file      : castor_wheel.hpp
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
#ifndef HW_COMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_CASTOR_WHEEL_HPP_
#define HW_COMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_CASTOR_WHEEL_HPP_

/* Includes ------------------------------------------------------------------*/
#include "wheel_base.hpp"

namespace hello_world
{
namespace chassis_ik_solver
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
HW_OPTIMIZE_O2_START

class CastorWheel : public Wheel
{
 public:
  explicit CastorWheel(const WheelParams& params) : Wheel(params) {}
  CastorWheel(const CastorWheel&) = default;
  CastorWheel& operator=(const CastorWheel&) = default;
  CastorWheel(CastorWheel&&) = default;
  CastorWheel& operator=(CastorWheel&&) = default;

  virtual ~CastorWheel(void) = default;

  IkSolveStatus ikSolve(
      const MoveVec& v, IkSolveRes* res_ptr,
      const float* theta_vel_fdb_ptr) override;
  virtual void setDCaster(float d) { params_.d_castor = fabsf(d); }

 private:
  float d_beta;
  uint32_t last_tick_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace chassis_ik_solver
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_CASTOR_WHEEL_HPP_ */
