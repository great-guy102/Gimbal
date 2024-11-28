/**
 *******************************************************************************
 * @file      : wheel_math.hpp
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
#ifndef HWCOMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_WHEEL_MATH_HPP_
#define HWCOMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_WHEEL_MATH_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>

#include "system.hpp"

#ifdef CHASSIS_IKSOLVER_USE_ARM_MATH
#include "arm_math.h"
#endif /* CHASSIS_IKSOLVER_USE_ARM_MATH */

namespace hello_world
{
namespace chassis_ik_solver
{
namespace internal
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_START

/**
 * @brief 计算角度的正弦和余弦值
 * @param ang 输入的角度，弧度制
 * @param sin_res 正弦值的输出
 * @param cos_res 余弦值的输出
 */
inline void sin_cos(float ang, float* p_sin_res, float* p_cos_res)
{
#ifndef CHASSIS_IKSOLVER_USE_ARM_MATH
  *p_sin_res = sinf(ang);
  *p_cos_res = cosf(ang);
#else
  arm_sin_cos_f32(ang / PI * 180, p_sin_res, p_cos_res);
#endif
}

/**
 * @brief 计算角度的正弦值
 * @param ang 输入的角度，弧度制
 * @return 返回角度的正弦值
 */
inline float sin(float ang)
{
#ifndef CHASSIS_IKSOLVER_USE_ARM_MATH
  return sinf(ang);
#else
  return arm_sin_f32(ang);
#endif
}

/**
 * @brief 计算角度的余弦值
 * @param ang 输入的角度，弧度制
 * @return 返回角度的余弦值
 */
inline float cos(float ang)
{
#ifndef CHASSIS_IKSOLVER_USE_ARM_MATH
  return cosf(ang);
#else
  return arm_cos_f32(ang);
#endif
}

/**
 * @brief 计算两个数的反正切值
 * @param y 输入的 y 值
 * @param x 输入的 x 值
 * @return 返回 y/x 的反正切值，弧度制
 */
inline float atan2(float y, float x)
{
#ifndef CHASSIS_IKSOLVER_USE_ARM_MATH
  return atan2f(y, x);
#else
  float res_ptr;
  arm_atan2_f32(y, x, &res_ptr);
  return res_ptr;
#endif
}

/**
 * @brief 计算两个向量的点积
 * @param p_vec1 输入的第一个向量
 * @param p_vec2 输入的第二个向量
 * @param count 向量的长度
 * @return 返回两个向量的点积
 */
inline float VecDot(const float* p_vec1, const float* p_vec2, size_t count)
{
  float res_ptr = 0;
#ifndef CHASSIS_IKSOLVER_USE_ARM_MATH
  for (size_t i = 0; i < count; i++) {
    res_ptr += p_vec1[i] * p_vec2[i];
  }
#else
  arm_dot_prod_f32(p_vec1, p_vec2, count, &res_ptr);
#endif
  return res_ptr;
}

/**
 * @brief 计算一个数的平方根
 * @param x 输入的数
 * @return 返回 x 的平方根
 */
inline float sqrt(float x)
{
#ifndef CHASSIS_IKSOLVER_USE_ARM_MATH
  return sqrtf(x);
#else
  float res_ptr;
  arm_sqrt_f32(x, &res_ptr);
  return res_ptr;
#endif
}

/**
 * @brief 检查一个向量是否为零向量
 * @param p_vec 输入的向量
 * @param count 向量的长度
 * @return 如果向量为零向量，返回 true，否则返回 false
 */
inline static bool IsZeroVec(const float* p_vec, size_t count)
{
  for (size_t i = 0; i < count; i++) {
    if (p_vec[i] < -0.001f || 0.001f < p_vec[i]) {
      return false;
    }
  }
  return true;
}

HW_OPTIMIZE_O2_END
}  // namespace internal
}  // namespace chassis_ik_solver
}  // namespace hello_world

#endif /* HWCOMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_WHEEL_MATH_HPP_ */
