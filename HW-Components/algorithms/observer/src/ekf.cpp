/**
 *******************************************************************************
 * @file      : ekf.cpp
 * @brief     : 扩展卡尔曼滤波器
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-12-27      Caikunzhen      1. 完成初版编写（未测试）
 *  V1.0.0      2004-07-13      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  该滤波器有发散的可能，使用时请谨慎
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ekf.hpp"

#include <cstring>

#include "assert.hpp"

namespace hello_world
{
namespace observer
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

HW_OPTIMIZE_O2_START

Ekf::Ekf(const Config& config)
    : Observer(config.x_dim, config.z_dim, config.u_dim),
      f_(config.f),
      h_(config.h),
      GetJ_F_(config.GetJ_F),
      GetJ_H_(config.GetJ_H)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(config.f != nullptr, "f must not be nullptr");
  HW_ASSERT(config.h != nullptr, "h must not be nullptr");
  HW_ASSERT(config.GetJ_F != nullptr, "GetJ_F must not be nullptr");
  HW_ASSERT(config.GetJ_H != nullptr, "GetJ_H must not be nullptr");
  HW_ASSERT(config.Q != nullptr, "Q must not be nullptr");
  HW_ASSERT(config.R != nullptr, "R must not be nullptr");
#pragma endregion

  Q_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  memcpy(Q_, config.Q, x_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&Q_mat_, x_dim_, x_dim_, Q_);

  R_ = Allocator<float>::allocate(z_dim_ * z_dim_);
  memcpy(R_, config.R, z_dim_ * z_dim_ * sizeof(float));
  arm_mat_init_f32(&R_mat_, z_dim_, z_dim_, R_);

  P_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  if (config.P != nullptr) {
    memcpy(P_, config.P, x_dim_ * x_dim_ * sizeof(float));
  } else {
    memset(P_, 0, x_dim_ * x_dim_ * sizeof(float));
  }
  arm_mat_init_f32(&P_mat_, x_dim_, x_dim_, P_);

  if (config.x0 != nullptr) {
    setX0(config.x0);
  }
}

Ekf::Ekf(const Ekf& other)
    : Observer(other.x_dim_, other.z_dim_, other.u_dim_),
      f_(other.f_),
      h_(other.h_),
      GetJ_F_(other.GetJ_F_),
      GetJ_H_(other.GetJ_H_)
{
  Q_ = Allocator<float>::allocate(x_dim_);
  memcpy(Q_, other.Q_, x_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&Q_mat_, x_dim_, x_dim_, Q_);

  R_ = Allocator<float>::allocate(z_dim_);
  memcpy(R_, other.R_, z_dim_ * z_dim_ * sizeof(float));
  arm_mat_init_f32(&R_mat_, z_dim_, z_dim_, R_);

  P_ = Allocator<float>::allocate(x_dim_);
  memcpy(P_, other.P_, x_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&P_mat_, x_dim_, x_dim_, P_);

  memcpy(x_hat_, other.x_hat_, x_dim_ * sizeof(float));
}

Ekf& Ekf::operator=(const Ekf& other)
{
  if (this == &other) {
    return *this;
  }

  if (P_ != nullptr) {
    Allocator<float>::deallocate(P_, x_dim_ * x_dim_);
  }

  if (Q_ != nullptr) {
    Allocator<float>::deallocate(Q_, x_dim_ * x_dim_);
  }

  if (R_ != nullptr) {
    Allocator<float>::deallocate(R_, z_dim_ * z_dim_);
  }

  Observer::operator=(other);

  Q_ = Allocator<float>::allocate(x_dim_);
  memcpy(Q_, other.Q_, x_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&Q_mat_, x_dim_, x_dim_, Q_);

  R_ = Allocator<float>::allocate(z_dim_);
  memcpy(R_, other.R_, z_dim_ * z_dim_ * sizeof(float));
  arm_mat_init_f32(&R_mat_, z_dim_, z_dim_, R_);

  P_ = Allocator<float>::allocate(x_dim_);
  memcpy(P_, other.P_, x_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&P_mat_, x_dim_, x_dim_, P_);

  memcpy(x_hat_, other.x_hat_, x_dim_ * sizeof(float));

  return *this;
}

Ekf::Ekf(Ekf&& other)
    : Observer(std::move(other)),
      f_(other.f_),
      h_(other.h_),
      GetJ_F_(other.GetJ_F_),
      GetJ_H_(other.GetJ_H_)
{
  Q_ = other.Q_;
  arm_mat_init_f32(&Q_mat_, x_dim_, x_dim_, Q_);
  other.Q_ = nullptr;

  R_ = other.R_;
  arm_mat_init_f32(&R_mat_, z_dim_, z_dim_, R_);
  other.R_ = nullptr;

  P_ = other.P_;
  arm_mat_init_f32(&P_mat_, x_dim_, x_dim_, P_);
  other.P_ = nullptr;

  memcpy(x_hat_, other.x_hat_, x_dim_ * sizeof(float));
}

Ekf& Ekf::operator=(Ekf&& other)
{
  if (this == &other) {
    return *this;
  }

  if (P_ != nullptr) {
    Allocator<float>::deallocate(P_, x_dim_ * x_dim_);
  }

  if (Q_ != nullptr) {
    Allocator<float>::deallocate(Q_, x_dim_ * x_dim_);
  }

  if (R_ != nullptr) {
    Allocator<float>::deallocate(R_, z_dim_ * z_dim_);
  }

  Observer::operator=(std::move(other));

  Q_ = other.Q_;
  arm_mat_init_f32(&Q_mat_, x_dim_, x_dim_, Q_);
  other.Q_ = nullptr;

  R_ = other.R_;
  arm_mat_init_f32(&R_mat_, z_dim_, z_dim_, R_);
  other.R_ = nullptr;

  P_ = other.P_;
  arm_mat_init_f32(&P_mat_, x_dim_, x_dim_, P_);
  other.P_ = nullptr;

  memcpy(x_hat_, other.x_hat_, x_dim_ * sizeof(float));

  return *this;
}

Ekf::~Ekf(void)
{
  Allocator<float>::deallocate(Q_, x_dim_);
  Allocator<float>::deallocate(R_, z_dim_);
  Allocator<float>::deallocate(P_, x_dim_);
}

void Ekf::calc(const float u[], const float z[])
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(u != nullptr, "u must not be nullptr");
  HW_ASSERT(z != nullptr, "z must not be nullptr");
#pragma endregion

  float* x_hat_bar = Allocator<float>::allocate(x_dim_);
  float* P_bar = Allocator<float>::allocate(x_dim_ * x_dim_);
  arm_matrix_instance_f32 P_bar_mat;
  arm_mat_init_f32(&P_bar_mat, x_dim_, x_dim_, P_bar);

  /* 预测 */
  predict(u, x_hat_bar, P_bar_mat);

  /* 校正 */
  update(z, x_hat_bar, P_bar_mat);

  Allocator<float>::deallocate(P_bar, x_dim_ * x_dim_);
  Allocator<float>::deallocate(x_hat_bar, x_dim_);
}

void Ekf::setX0(const float x0[])
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(x0 != nullptr, "x0 must not be nullptr");
#pragma endregion

  memcpy(x_hat_, x0, x_dim_ * sizeof(float));
}

void Ekf::init(const Config& config)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(config.f != nullptr, "f must not be nullptr");
  HW_ASSERT(config.h != nullptr, "h must not be nullptr");
  HW_ASSERT(config.GetJ_F != nullptr, "GetJ_F must not be nullptr");
  HW_ASSERT(config.GetJ_H != nullptr, "GetJ_H must not be nullptr");
  HW_ASSERT(config.Q != nullptr, "Q must not be nullptr");
  HW_ASSERT(config.R != nullptr, "R must not be nullptr");
#pragma endregion

  if (Q_ != nullptr) {
    Allocator<float>::deallocate(Q_, x_dim_ * x_dim_);
  }

  if (R_ != nullptr) {
    Allocator<float>::deallocate(R_, z_dim_ * z_dim_);
  }

  if (P_ != nullptr) {
    Allocator<float>::deallocate(P_, x_dim_ * x_dim_);
  }

  Observer::init(config.x_dim, config.z_dim, config.u_dim);

  f_ = config.f;
  h_ = config.h;
  GetJ_F_ = config.GetJ_F;
  GetJ_H_ = config.GetJ_H;

  Q_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  memcpy(Q_, config.Q, x_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&Q_mat_, x_dim_, x_dim_, Q_);

  R_ = Allocator<float>::allocate(z_dim_ * z_dim_);
  memcpy(R_, config.R, z_dim_ * z_dim_ * sizeof(float));
  arm_mat_init_f32(&R_mat_, z_dim_, z_dim_, R_);

  P_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  if (config.P != nullptr) {
    memcpy(P_, config.P, x_dim_ * x_dim_ * sizeof(float));
  } else {
    memset(P_, 0, x_dim_ * x_dim_ * sizeof(float));
  }
  arm_mat_init_f32(&P_mat_, x_dim_, x_dim_, P_);

  if (config.x0 != nullptr) {
    setX0(config.x0);
  }
}

void Ekf::predict(const float u[], float x_hat_bar[],
                  arm_matrix_instance_f32& P_bar_mat) const
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(u != nullptr, "u must not be nullptr");
  HW_ASSERT(x_hat_bar != nullptr, "x_hat_bar must not be nullptr");
#pragma endregion

  /* x_hat_bar = f(x_hat_, u) */
  f_(x_hat_, u, x_hat_bar);

  /* P_bar = J_F * P * J_F' + Q */
  float* J_F = Allocator<float>::allocate(x_dim_ * x_dim_);
  float* mat_tmp1 = Allocator<float>::allocate(x_dim_ * x_dim_);
  float* mat_tmp2 = Allocator<float>::allocate(x_dim_ * x_dim_);
  GetJ_F_(x_hat_, u, J_F);
  arm_matrix_instance_f32 J_F_mat, mat_tmp_mat1, mat_tmp_mat2;
  arm_mat_init_f32(&J_F_mat, x_dim_, x_dim_, J_F);
  arm_mat_init_f32(&mat_tmp_mat1, x_dim_, x_dim_, mat_tmp1);
  arm_mat_init_f32(&mat_tmp_mat2, x_dim_, x_dim_, mat_tmp2);

  arm_mat_trans_f32(&J_F_mat, &mat_tmp_mat1);                // J_F'
  arm_mat_mult_f32(&P_mat_, &mat_tmp_mat1, &mat_tmp_mat2);   // P * J_F'
  arm_mat_mult_f32(&J_F_mat, &mat_tmp_mat2, &mat_tmp_mat1);  // J_F * P * J_F'
  arm_mat_add_f32(&mat_tmp_mat1, &Q_mat_, &P_bar_mat);       // J_F * P * J_F' + Q

  Allocator<float>::deallocate(mat_tmp2, x_dim_ * x_dim_);
  Allocator<float>::deallocate(mat_tmp1, x_dim_ * x_dim_);
  Allocator<float>::deallocate(J_F, x_dim_ * x_dim_);
}

void Ekf::update(const float z[], const float x_hat_bar[],
                 const arm_matrix_instance_f32& P_bar_mat)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(z != nullptr, "z must not be nullptr");
  HW_ASSERT(x_hat_bar != nullptr, "x_hat_bar must not be nullptr");
#pragma endregion

  float* K = Allocator<float>::allocate(x_dim_ * z_dim_);
  float* J_H = Allocator<float>::allocate(z_dim_ * x_dim_);
  GetJ_H_(x_hat_bar, J_H);
  arm_matrix_instance_f32 K_mat, J_H_mat;
  arm_mat_init_f32(&K_mat, x_dim_, z_dim_, K);
  arm_mat_init_f32(&J_H_mat, z_dim_, x_dim_, J_H);

  /* K = P_bar * J_H' * (J_H * P_bar * J_H' + R)^(-1) */
  {
    float* mat_tmp1 = Allocator<float>::allocate(x_dim_ * z_dim_);
    float* mat_tmp2 = Allocator<float>::allocate(x_dim_ * z_dim_);
    float* mat_tmp3 = Allocator<float>::allocate(z_dim_ * z_dim_);
    float* mat_tmp4 = Allocator<float>::allocate(z_dim_ * z_dim_);
    arm_matrix_instance_f32 mat_tmp_mat1, mat_tmp_mat2, mat_tmp_mat3,
        mat_tmp_mat4;
    arm_mat_init_f32(&mat_tmp_mat1, x_dim_, z_dim_, mat_tmp1);
    arm_mat_init_f32(&mat_tmp_mat2, x_dim_, z_dim_, mat_tmp2);
    arm_mat_init_f32(&mat_tmp_mat3, z_dim_, z_dim_, mat_tmp3);
    arm_mat_init_f32(&mat_tmp_mat4, z_dim_, z_dim_, mat_tmp4);

    arm_mat_trans_f32(&J_H_mat, &mat_tmp_mat1);                  // J_H'
    arm_mat_mult_f32(&P_bar_mat, &mat_tmp_mat1, &mat_tmp_mat2);  // P_bar * J_H'
    arm_mat_mult_f32(&J_H_mat, &mat_tmp_mat2, &mat_tmp_mat3);    // J_H * P_bar * J_H'
    arm_mat_add_f32(&mat_tmp_mat3, &R_mat_, &mat_tmp_mat4);      // J_H * P_bar * J_H' + R
    arm_mat_inverse_f32(&mat_tmp_mat4, &mat_tmp_mat3);           // (J_H * P_bar * J_H' + R)^(-1)
    arm_mat_mult_f32(&mat_tmp_mat2, &mat_tmp_mat3, &K_mat);      // P_bar * J_H' * (J_H * P_bar * J_H' + R)^(-1)

    Allocator<float>::deallocate(mat_tmp4, z_dim_ * z_dim_);
    Allocator<float>::deallocate(mat_tmp3, z_dim_ * z_dim_);
    Allocator<float>::deallocate(mat_tmp2, x_dim_ * z_dim_);
    Allocator<float>::deallocate(mat_tmp1, x_dim_ * z_dim_);
  }

  /* x_hat_ = x_hat_bar + K * (z - h(x_hat_bar)) */
  {
    float* z_hat = Allocator<float>::allocate(z_dim_);
    float* tmp1 = Allocator<float>::allocate(z_dim_);
    float* tmp2 = Allocator<float>::allocate(x_dim_);
    h_(x_hat_bar, z_hat);                          // h(x_hat_bar)
    arm_sub_f32(z, z_hat, tmp1, z_dim_);           // z - h(x_hat_bar)
    arm_mat_vec_mult_f32(&K_mat, tmp1, tmp2);      // K * (z - h(x_hat_bar))
    arm_add_f32(x_hat_bar, tmp2, x_hat_, x_dim_);  // x_hat_bar + K * (z - h(x_hat_bar))

    Allocator<float>::deallocate(tmp2, x_dim_);
    Allocator<float>::deallocate(tmp1, z_dim_);
    Allocator<float>::deallocate(z_hat, z_dim_);
  }

  /* P = (I - K * J_H) * P_bar */
  {
    float* mat_tmp1 = Allocator<float>::allocate(x_dim_ * x_dim_);
    float* mat_tmp2 = Allocator<float>::allocate(x_dim_ * x_dim_);
    float* I = Allocator<float>::allocate(x_dim_ * x_dim_);
    memset(I, 0, x_dim_ * x_dim_ * sizeof(float));
    for (size_t i = 0; i < x_dim_; i++) {
      I[i * x_dim_ + i] = 1.0f;
    }

    arm_matrix_instance_f32 mat_tmp_mat1, mat_tmp_mat2, I_mat;
    arm_mat_init_f32(&mat_tmp_mat1, x_dim_, x_dim_, mat_tmp1);
    arm_mat_init_f32(&mat_tmp_mat2, x_dim_, x_dim_, mat_tmp2);
    arm_mat_init_f32(&I_mat, x_dim_, x_dim_, I);

    arm_mat_mult_f32(&K_mat, &J_H_mat, &mat_tmp_mat1);      // K * J_H
    arm_mat_sub_f32(&I_mat, &mat_tmp_mat1, &mat_tmp_mat2);  // I - K * J_H
    arm_mat_mult_f32(&mat_tmp_mat2, &P_bar_mat, &P_mat_);   // (I - K * J_H) * P_bar

    Allocator<float>::deallocate(I, x_dim_ * x_dim_);
    Allocator<float>::deallocate(mat_tmp2, x_dim_ * x_dim_);
    Allocator<float>::deallocate(mat_tmp1, x_dim_ * x_dim_);
  }

  Allocator<float>::deallocate(J_H, z_dim_ * x_dim_);
  Allocator<float>::deallocate(K, x_dim_ * z_dim_);
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace observer
}  // namespace hello_world