/**
 *******************************************************************************
 * @file      : serial_rod.cpp
 * @brief     :串联连杆机构运动学求解
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2024-08-07      Caikunzhen      1. 未测试版本
 *******************************************************************************
 * @attention :
 *  只提供串联连杆机构的正运动学求解和雅克比矩阵求解，当有逆运动学求解需求时，请继承该类
 *  并实现逆运动学求解方法
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "serial_rod.hpp"

#include "arm_math.h"
#include "assert.hpp"
#include "base.hpp"

namespace hello_world
{
namespace serial_rod
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
HW_OPTIMIZE_O2_START

SerialRod::SerialRod(JointParams joint_params_arr[], size_t joint_num)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(joint_params_arr != nullptr, "joint_params_arr is nullptr");
  HW_ASSERT(joint_num > 0, "joint_num is less than 1");
#pragma endregion

  joint_num_ = joint_num;
  joint_params_arr_ = new JointParams[joint_num_];
  for (size_t i = 0; i < joint_num_; i++) {
    joint_params_arr_[i] = joint_params_arr[i];
  }
}

SerialRod::SerialRod(const SerialRod& other)
{
  joint_num_ = other.joint_num_;
  joint_params_arr_ = new JointParams[joint_num_];
  for (size_t i = 0; i < joint_num_; i++) {
    joint_params_arr_[i] = other.joint_params_arr_[i];
  }
}

SerialRod& SerialRod::operator=(const SerialRod& other)
{
  if (this == &other) {
    return *this;
  }

  if (joint_params_arr_ != nullptr) {
    delete[] joint_params_arr_;
  }

  joint_num_ = other.joint_num_;
  joint_params_arr_ = new JointParams[joint_num_];
  for (size_t i = 0; i < joint_num_; i++) {
    joint_params_arr_[i] = other.joint_params_arr_[i];
  }

  return *this;
}

SerialRod::SerialRod(SerialRod&& other)
{
  joint_num_ = other.joint_num_;
  joint_params_arr_ = other.joint_params_arr_;

  other.joint_num_ = 0;
  other.joint_params_arr_ = nullptr;
}

SerialRod& SerialRod::operator=(SerialRod&& other)
{
  if (this == &other) {
    return *this;
  }

  if (joint_params_arr_ != nullptr) {
    delete[] joint_params_arr_;
  }

  joint_num_ = other.joint_num_;
  joint_params_arr_ = other.joint_params_arr_;

  other.joint_num_ = 0;
  other.joint_params_arr_ = nullptr;

  return *this;
}

SerialRod::~SerialRod(void)
{
  delete[] joint_params_arr_;
  joint_params_arr_ = nullptr;
}

void SerialRod::init(JointParams joint_params_arr[], size_t joint_num)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(joint_params_arr != nullptr, "joint_params_arr is nullptr");
  HW_ASSERT(joint_num > 0, "joint_num is less than 1");
#pragma endregion

  if (joint_params_arr_ != nullptr) {
    delete[] joint_params_arr_;
  }

  joint_num_ = joint_num;
  joint_params_arr_ = new JointParams[joint_num_];
  for (size_t i = 0; i < joint_num_; i++) {
    joint_params_arr_[i] = joint_params_arr[i];
  }
}

void SerialRod::fwdKinematics(const float q[], Eigen::Isometry3f& tf_n20) const
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(q != nullptr, "q is nullptr");
#pragma endregion

  tf_n20 = Eigen::Isometry3f::Identity();
  for (size_t i = 0; i < joint_num_; i++) {
    Eigen::Isometry3f tf_i2i_1;
    JointParams joint_params = joint_params_arr_[i];

    if (joint_params.type == JointType::kRevolute) {
      joint_params.theta += q[i];
    } else {
      joint_params.d += q[i];
    }
    calcIsometry(joint_params, tf_i2i_1);
    tf_n20 = tf_n20 * tf_i2i_1;
  }
}

void SerialRod::getTf2BaseArr(const float q[], Eigen::Isometry3f tf_arr[]) const
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(q != nullptr, "q is nullptr");
  HW_ASSERT(tf_arr != nullptr, "tf_arr is nullptr");
#pragma endregion

  JointParams joint_params = joint_params_arr_[0];
  if (joint_params.type == JointType::kRevolute) {
    joint_params.theta += q[0];
  } else {
    joint_params.d += q[0];
  }
  calcIsometry(joint_params, tf_arr[0]);

  for (size_t i = 1; i < joint_num_; i++) {
    joint_params = joint_params_arr_[i];
    if (joint_params.type == JointType::kRevolute) {
      joint_params.theta += q[i];
    } else {
      joint_params.d += q[i];
    }

    Eigen::Isometry3f tf_i2i_1;
    calcIsometry(joint_params, tf_i2i_1);
    tf_arr[i] = tf_arr[i - 1] * tf_i2i_1;
  }
}

void SerialRod::jacobian(const float q[], Eigen::MatrixXf& jacobian) const
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(q != nullptr, "q is nullptr");
  HW_ASSERT(jacobian.rows() == 6, "jacobian.rows() is not equal to 6");
  HW_ASSERT((size_t)jacobian.cols() == joint_num_,
            "jacobian.cols() is not equal to joint_num_");
#pragma endregion

  Eigen::Isometry3f* tf_arr = new Eigen::Isometry3f[joint_num_];
  getTf2BaseArr(q, tf_arr);

  Eigen::Vector3f p_n = tf_arr[joint_num_ - 1].translation();
  for (size_t i = 0; i < joint_num_; i++) {
    Eigen::Vector3f z = tf_arr[i].linear().block<3, 1>(0, 2);
    if (joint_params_arr_[i].type == JointType::kRevolute) {
      Eigen::Vector3f p = tf_arr[i].translation();
      Eigen::Vector3f jv = z.cross(p_n - p);
      jacobian.block<3, 1>(0, i) = jv;
      jacobian.block<3, 1>(3, i) = z;
    } else {
      jacobian.block<3, 1>(0, i) = z;
      jacobian.block<3, 1>(3, i) = Eigen::Vector3f::Zero();
    }
  }
}

void SerialRod::calcIsometry(
    const JointParams& joint_params, Eigen::Isometry3f& tf) const
{
  const float& a = joint_params.a;
  const float& alpha = joint_params.alpha;
  const float& d = joint_params.d;
  const float& theta = joint_params.theta;

  float cAlpha, sAlpha, cTheta, sTheta;
  arm_sin_cos_f32(Rad2Deg(alpha), &sAlpha, &cAlpha);
  arm_sin_cos_f32(Rad2Deg(theta), &sTheta, &cTheta);

  Eigen::Matrix3f rot;
  rot << cTheta, -sTheta * cAlpha, sTheta * sAlpha,
      sTheta, cTheta * cAlpha, -cTheta * sAlpha,
      0, sAlpha, cAlpha;

  Eigen::Vector3f trans(a * cTheta, a * sTheta, d);

  tf.setIdentity();
  tf.rotate(rot);
  tf.pretranslate(trans);
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace serial_rod
}  // namespace hello_world
