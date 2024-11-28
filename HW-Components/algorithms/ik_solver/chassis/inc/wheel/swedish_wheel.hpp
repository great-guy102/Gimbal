/**
 *******************************************************************************
 * @file      : swedish_wheel.hpp
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
#ifndef HW_COMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_SWEDISH_WHEEL_HPP_
#define HW_COMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_SWEDISH_WHEEL_HPP_

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

class SwedishWheel : public Wheel
{
 public:
  typedef Wheel::OptMask OptMask;

  explicit SwedishWheel(const WheelParams& params) : Wheel(params)
  {
    setGamma(params.gamma);
  }
  SwedishWheel(const SwedishWheel&) = default;
  SwedishWheel& operator=(const SwedishWheel&) = default;
  SwedishWheel(SwedishWheel&&) = default;
  SwedishWheel& operator=(SwedishWheel&&) = default;

  virtual ~SwedishWheel(void) = default;

  virtual IkSolveStatus ikSolve(
      const MoveVec& v, IkSolveRes* res_ptr,
      const float* theta_vel_fdb_ptr = nullptr) override
  {
    IkSolveStatus status = kIkSolveStatusOk;

    if (params_.radius < 0.001) {
      SetBits(kIkSolveStatusRadiusTooSmall, status);
      params_.radius = 0.001;
    }

    float aby = params_.theta_vel_fdb + M_PI_2 + getGamma();
    float by = aby - alpha_;
    float sin_aby = 0, cos_aby = 0, sin_by = 0, cos_by = 0;
    internal::sin_cos(aby, &sin_aby, &cos_aby);
    internal::sin_cos(by, &sin_by, &cos_by);

    float J1[3] = {sin_aby, -cos_aby, -l_ * cos_by};
    // float C1[3] = {cos_aby, sin_aby, l_ * sin_by};

    // 无侧滑约束求解
    // float no_side_slip_val = internal::VecDot(C1, v.vec(), 3);
    iksolve_res_.is_no_side_slip = true;
    iksolve_res_.theta_vel_ref = params_.theta_vel_fdb;

    // 滚动约束求解
    float cos_gamma = internal::cos(getGamma());
    float rot_spd = 0;
    if (fabs(cos_gamma) > 0.001) {
      rot_spd = internal::VecDot(J1, v.vec(), 3) / (params_.radius * cos_gamma);
    }
    iksolve_res_.rot_spt = rot_spd;
    // 记录结果
    if (res_ptr != nullptr) {
      *res_ptr = iksolve_res_;
    } else {
      SetBits(kIkSolveStatusFailReturn, status);
    }
    return status;
  }

  virtual void setGamma(float gamma) override { params_.gamma = gamma; }
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace chassis_ik_solver
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_SWEDISH_WHEEL_HPP_ */
