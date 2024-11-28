/**
 *******************************************************************************
 * @file      : wheel_vec.hpp
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
#ifndef HWCOMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_WHEEL_VEC_HPP_
#define HWCOMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_WHEEL_VEC_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>

#include "allocator.hpp"
#include "system.hpp"
#include "wheel_math.hpp"

namespace hello_world
{
namespace chassis_ik_solver
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
HW_OPTIMIZE_O2_START

class MoveVec : public MemMgr
{
 public:
  explicit MoveVec(float x = 0, float y = 0, float w = 0)
  {
    data_[0] = x, data_[1] = y, data_[2] = w;
  }
  MoveVec(const MoveVec&) = default;
  MoveVec& operator=(const MoveVec&) = default;
  MoveVec(MoveVec&&) = default;
  MoveVec& operator=(MoveVec&&) = default;

  virtual ~MoveVec(void) = default;

  float& x(void) { return data_[0]; }
  float& y(void) { return data_[1]; }
  float& w(void) { return data_[2]; }
  float x(void) const { return data_[0]; }
  float y(void) const { return data_[1]; }
  float w(void) const { return data_[2]; }

  float* vec(void) { return data_; }
  const float* vec(void) const { return data_; }

  float& operator[](size_t idx) { return data_[idx]; }
  const float& operator[](size_t idx) const { return data_[idx]; }

  void rotate(float ang, MoveVec* res_ptr = nullptr) const
  {
    if (res_ptr == nullptr) {
      return;
    }

    float c, s;
    internal::sin_cos(ang, &s, &c);

    res_ptr->x() = c * x() + s * y();
    res_ptr->y() = -s * x() + c * y();
    res_ptr->w() = w();  // w is not changed by rotation.
  }

 private:
  float data_[3] = {0};  // 矢量数据 [x, y, w]
};

class PosVec : public MemMgr
{
 public:
  explicit PosVec(float x = 0, float y = 0) { data_[0] = x, data_[1] = y; };
  PosVec(const PosVec&) = default;
  PosVec& operator=(const PosVec&) = default;
  PosVec(PosVec&&) = default;
  PosVec& operator=(PosVec&&) = default;

  virtual ~PosVec(void) = default;

  float& x(void) { return data_[0]; }
  float& y(void) { return data_[1]; }
  float x(void) const { return data_[0]; }
  float y(void) const { return data_[1]; }

  float* vec(void) { return data_; }
  const float* vec(void) const { return data_; }

  float operator[](size_t idx) const { return data_[idx]; }
  float& operator[](size_t idx) { return data_[idx]; }

  float norm(void) const
  {
    return internal::sqrt(internal::VecDot(data_, data_, 2));
  }

  float ang(void) const
  {
    return internal::atan2(y(), x());
  }

  float dist(const PosVec& other) const { return (*this - other).norm(); }

  PosVec operator+(const PosVec& other) const
  {
    return PosVec(data_[0] + other.data_[0], data_[1] + other.data_[1]);
  }
  PosVec operator-(const PosVec& other) const
  {
    return PosVec(data_[0] - other.data_[0], data_[1] - other.data_[1]);
  }

  PosVec& operator+=(const PosVec& other)
  {
    data_[0] += other.data_[0];
    data_[1] += other.data_[1];
    return *this;
  }
  PosVec& operator-=(const PosVec& other)
  {
    data_[0] -= other.data_[0];
    data_[1] -= other.data_[1];
    return *this;
  }

 private:
  float data_[2] = {0};  ///< 位置矢量数据 [x, y]
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace chassis_ik_solver
}  // namespace hello_world

#endif /* HWCOMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_WHEEL_VEC_HPP_ */
