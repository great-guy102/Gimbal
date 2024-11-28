/**
 *******************************************************************************
 * @file      : spherical_wheel.hpp
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
#ifndef HW_COMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_SPHERICAL_WHEEL_HPP_
#define HW_COMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_SPHERICAL_WHEEL_HPP_

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

class SphericalWheel : public Wheel
{
 public:
  enum OptMask {
    kOptMaskNone = 0,
    kOptMaskUseThetaVelFdb = 1u << 0,    ///< 使用反馈的车轮转角计算车轮转速
    kOptMaskMinThetaVelDelta = 1u << 1,  ///< 通过转速反向使车轮转角变化最小
    /*/< 当使用期望的车轮转角计算车轮转速时，对转速乘以 sin 函数 */
    kOptMaskCosRotSpd = 1u << 2,
    kOptMaskAsServer = 1u << 3,  ///< 作为随动轮使用
  };
  explicit SphericalWheel(const WheelParams& params) : Wheel(params) {}
  SphericalWheel(const SphericalWheel&) = default;
  SphericalWheel& operator=(const SphericalWheel&) = default;
  SphericalWheel(SphericalWheel&&) = default;
  SphericalWheel& operator=(SphericalWheel&&) = default;

  virtual ~SphericalWheel(void) = default;

  virtual IkSolveStatus ikSolve(
      const MoveVec& v, IkSolveRes* res_ptr,
      const float* theta_vel_fdb_ptr = nullptr) override
  {
    IkSolveStatus status = kIkSolveStatusOk;

    if (theta_vel_fdb_ptr != nullptr) {
      params_.theta_vel_fdb = *theta_vel_fdb_ptr;
    } else {
      SetBits(kIkSolveStatusLeakVelAngFdb, status);
    }

    if (params_.radius < 0.001) {
      SetBits(kIkSolveStatusRadiusTooSmall, status);
      params_.radius = 0.001;
    }

    float a = alpha_;
    float beta_fdb = getBeta(params_.theta_vel_fdb);
    float sin_a, cos_a;
    float sin_b_fdb, cos_b_fdb;
    internal::sin_cos(a, &sin_a, &cos_a);
    internal::sin_cos(beta_fdb, &sin_b_fdb, &cos_b_fdb);
    float var1 = v.x() * cos_a + v.y() * sin_a;
    float var2 = v.x() * sin_a - v.y() * cos_a - l_ * v.w();

    // 无侧滑约束求解
    float beta_ref = 0;
    if (!internal::IsZeroVec(v.vec(), 3)) {
      beta_ref = internal::atan2(var1, var2);
    }
    float theta_vel_delta = 0.0f;
    float rot_spd_dir = 1.0f;
    if (IsBitsSet(kOptMaskMinThetaVelDelta, params_.opt_mask)) {
      theta_vel_delta = beta_ref - beta_fdb;
      if (fabsf(theta_vel_delta) / M_PI_2 > 1) {
        beta_ref += PI;
        rot_spd_dir = -1.0f;
      }
    }
    iksolve_res_.theta_vel_ref = NormPeriodData(
        -PI, PI, beta_ref + alpha_ - M_PI_2);

    if (IsBitsSet(kOptMaskAsServer, params_.opt_mask)) {
      iksolve_res_.is_no_side_slip = true;
    } else {
      float no_side_slip_val = var1 * cos_b_fdb - var2 * sin_b_fdb;
      iksolve_res_.is_no_side_slip = (-0.001f < no_side_slip_val) &&
                                     (no_side_slip_val < 0.001f);
    }

    float sin_b, cos_b;
    float rot_spd = 0;

    if (IsBitsSet(kOptMaskUseThetaVelFdb, params_.opt_mask)) {
      sin_b = sin_b_fdb, cos_b = cos_b_fdb;
      rot_spd = (var1 * sin_b + var2 * cos_b) / params_.radius;
      rot_spd *= rot_spd_dir;
      // ! 可能还需要反向
    } else {
      internal::sin_cos(beta_ref, &sin_b, &cos_b);
      rot_spd = (var1 * sin_b + var2 * cos_b) / params_.radius;
      if (IsBitsSet(kOptMaskCosRotSpd, params_.opt_mask)) {
        rot_spd *= internal::cos(theta_vel_delta);
      } else {
        rot_spd *= rot_spd_dir;
      }
    }
    iksolve_res_.rot_spt = rot_spd;

    // TODO(ZhouShichan): 之后加一个启停优化
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

#endif /* HW_COMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_SPHERICAL_WHEEL_HPP_ */
