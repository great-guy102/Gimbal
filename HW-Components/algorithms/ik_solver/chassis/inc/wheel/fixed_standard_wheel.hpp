/**
 *******************************************************************************
 * @file      : fixed_standard_wheel.hpp
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
#ifndef HW_COMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_FIXED_STANDARD_WHEEL_HPP_
#define HW_COMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_FIXED_STANDARD_WHEEL_HPP_

/* Includes ------------------------------------------------------------------*/
#include "system.hpp"
#include "wheel_base.hpp"

namespace hello_world
{
namespace chassis_ik_solver
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
HW_OPTIMIZE_O2_START

class FixedStandardWheel : public Wheel
{
 public:
  typedef Wheel::OptMask OptMask;

  explicit FixedStandardWheel(const WheelParams& params) : Wheel(params) {}
  FixedStandardWheel(const FixedStandardWheel&) = default;
  FixedStandardWheel& operator=(const FixedStandardWheel&) = default;
  FixedStandardWheel(FixedStandardWheel&&) = default;
  FixedStandardWheel& operator=(FixedStandardWheel&&) = default;

  virtual ~FixedStandardWheel(void) = default;

  virtual IkSolveStatus ikSolve(
      const MoveVec& v, IkSolveRes* res_ptr,
      const float* theta_vel_fdb_ptr = nullptr) override
  {
    IkSolveStatus status = kIkSolveStatusOk;

    if (params_.radius < 0.001) {
      SetBits(kIkSolveStatusRadiusTooSmall, status);
      params_.radius = 0.001;
    }

    float aby = params_.theta_vel_fdb + M_PI_2;
    float by = aby - alpha_;
    float sin_aby = 0, cos_aby = 0, sin_by = 0, cos_by = 0;
    internal::sin_cos(aby, &sin_aby, &cos_aby);
    internal::sin_cos(by, &sin_by, &cos_by);

    float J1[3] = {sin_aby, -cos_aby, -l_ * cos_by};
    float C1[3] = {cos_aby, sin_aby, l_ * sin_by};

    // 无侧滑约束求解
    float no_side_slip_val = internal::VecDot(C1, v.vec(), 3);
    iksolve_res_.is_no_side_slip = (-0.001f < no_side_slip_val) &&
                                   (no_side_slip_val < 0.001f);
    iksolve_res_.theta_vel_ref = params_.theta_vel_fdb;

    // 滚动约束求解
    float rot_spd = internal::VecDot(J1, v.vec(), 3) / params_.radius;
    iksolve_res_.rot_spt = rot_spd;
    // 记录结果
    if (res_ptr != nullptr) {
      *res_ptr = iksolve_res_;
    } else {
      SetBits(kIkSolveStatusFailReturn, status);
    }
    return status;
  }
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace chassis_ik_solver
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_FIXED_STANDARD_WHEEL_HPP_ */
