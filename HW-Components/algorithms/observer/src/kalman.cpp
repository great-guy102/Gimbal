/**
 *******************************************************************************
 * @file      : kalman.cpp
 * @brief     : 卡尔曼滤波器
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-12-27      Caikunzhen      1. 完成初版编写（未测试）
 *  V1.0.0      2004-07-13      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "kalman.hpp"

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

Kalman::Kalman(const Config& config_d)
    : Observer(config_d.x_dim, config_d.z_dim, config_d.u_dim)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(config_d.F != nullptr, "F must not be nullptr");
  HW_ASSERT(config_d.B != nullptr, "B must not be nullptr");
  HW_ASSERT(config_d.H != nullptr, "H must not be nullptr");
  HW_ASSERT(config_d.Q != nullptr, "Q must not be nullptr");
  HW_ASSERT(config_d.R != nullptr, "R must not be nullptr");
#pragma endregion

  F_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  memcpy(F_, config_d.F, x_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&F_mat_, x_dim_, x_dim_, F_);

  B_ = Allocator<float>::allocate(x_dim_ * u_dim_);
  memcpy(B_, config_d.B, x_dim_ * u_dim_ * sizeof(float));
  arm_mat_init_f32(&B_mat_, x_dim_, u_dim_, B_);

  H_ = Allocator<float>::allocate(z_dim_ * x_dim_);
  memcpy(H_, config_d.H, z_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&H_mat_, z_dim_, x_dim_, H_);

  Q_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  memcpy(Q_, config_d.Q, x_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&Q_mat_, x_dim_, x_dim_, Q_);

  R_ = Allocator<float>::allocate(z_dim_ * z_dim_);
  memcpy(R_, config_d.R, z_dim_ * z_dim_ * sizeof(float));
  arm_mat_init_f32(&R_mat_, z_dim_, z_dim_, R_);

  P_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  if (config_d.P != nullptr) {
    memcpy(P_, config_d.P, x_dim_ * x_dim_ * sizeof(float));

  } else {
    memset(P_, 0, x_dim_ * x_dim_ * sizeof(float));
  }
  arm_mat_init_f32(&P_mat_, x_dim_, x_dim_, P_);

  if (config_d.x0 != nullptr) {
    setX0(config_d.x0);
  }
}

Kalman::Kalman(const Kalman& other)
    : Observer(other)
{
  F_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  memcpy(F_, other.F_, x_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&F_mat_, x_dim_, x_dim_, F_);

  B_ = Allocator<float>::allocate(x_dim_ * u_dim_);
  memcpy(B_, other.B_, x_dim_ * u_dim_ * sizeof(float));
  arm_mat_init_f32(&B_mat_, x_dim_, u_dim_, B_);

  H_ = Allocator<float>::allocate(z_dim_ * x_dim_);
  memcpy(H_, other.H_, z_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&H_mat_, z_dim_, x_dim_, H_);

  Q_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  memcpy(Q_, other.Q_, x_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&Q_mat_, x_dim_, x_dim_, Q_);

  R_ = Allocator<float>::allocate(z_dim_ * z_dim_);
  memcpy(R_, other.R_, z_dim_ * z_dim_ * sizeof(float));
  arm_mat_init_f32(&R_mat_, z_dim_, z_dim_, R_);

  P_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  memcpy(P_, other.P_, x_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&P_mat_, x_dim_, x_dim_, P_);
}

Kalman& Kalman::operator=(const Kalman& other)
{
  if (this == &other) {
    return *this;
  }

  if (F_ != nullptr) {
    Allocator<float>::deallocate(F_, x_dim_ * x_dim_);
  }

  if (B_ != nullptr) {
    Allocator<float>::deallocate(B_, x_dim_ * u_dim_);
  }

  if (H_ != nullptr) {
    Allocator<float>::deallocate(H_, z_dim_ * x_dim_);
  }

  if (Q_ != nullptr) {
    Allocator<float>::deallocate(Q_, x_dim_ * x_dim_);
  }

  if (R_ != nullptr) {
    Allocator<float>::deallocate(R_, z_dim_ * z_dim_);
  }

  if (P_ != nullptr) {
    Allocator<float>::deallocate(P_, x_dim_ * x_dim_);
  }

  Observer::operator=(other);

  F_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  memcpy(F_, other.F_, x_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&F_mat_, x_dim_, x_dim_, F_);

  B_ = Allocator<float>::allocate(x_dim_ * u_dim_);
  memcpy(B_, other.B_, x_dim_ * u_dim_ * sizeof(float));
  arm_mat_init_f32(&B_mat_, x_dim_, u_dim_, B_);

  H_ = Allocator<float>::allocate(z_dim_ * x_dim_);
  memcpy(H_, other.H_, z_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&H_mat_, z_dim_, x_dim_, H_);

  Q_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  memcpy(Q_, other.Q_, x_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&Q_mat_, x_dim_, x_dim_, Q_);

  R_ = Allocator<float>::allocate(z_dim_ * z_dim_);
  memcpy(R_, other.R_, z_dim_ * z_dim_ * sizeof(float));
  arm_mat_init_f32(&R_mat_, z_dim_, z_dim_, R_);

  P_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  memcpy(P_, other.P_, x_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&P_mat_, x_dim_, x_dim_, P_);

  return *this;
}

Kalman::Kalman(Kalman&& other)
    : Observer(std::move(other))
{
  F_ = other.F_;
  other.F_ = nullptr;
  arm_mat_init_f32(&F_mat_, x_dim_, x_dim_, F_);

  B_ = other.B_;
  other.B_ = nullptr;
  arm_mat_init_f32(&B_mat_, x_dim_, u_dim_, B_);

  H_ = other.H_;
  other.H_ = nullptr;
  arm_mat_init_f32(&H_mat_, z_dim_, x_dim_, H_);

  Q_ = other.Q_;
  other.Q_ = nullptr;
  arm_mat_init_f32(&Q_mat_, x_dim_, x_dim_, Q_);

  R_ = other.R_;
  other.R_ = nullptr;
  arm_mat_init_f32(&R_mat_, z_dim_, z_dim_, R_);

  P_ = other.P_;
  other.P_ = nullptr;
  arm_mat_init_f32(&P_mat_, x_dim_, x_dim_, P_);
}

Kalman& Kalman::operator=(Kalman&& other)
{
  if (this == &other) {
    return *this;
  }

  if (F_ != nullptr) {
    Allocator<float>::deallocate(F_, x_dim_ * x_dim_);
  }

  if (B_ != nullptr) {
    Allocator<float>::deallocate(B_, x_dim_ * u_dim_);
  }

  if (H_ != nullptr) {
    Allocator<float>::deallocate(H_, z_dim_ * x_dim_);
  }

  if (Q_ != nullptr) {
    Allocator<float>::deallocate(Q_, x_dim_ * x_dim_);
  }

  if (R_ != nullptr) {
    Allocator<float>::deallocate(R_, z_dim_ * z_dim_);
  }

  if (P_ != nullptr) {
    Allocator<float>::deallocate(P_, x_dim_ * x_dim_);
  }

  Observer::operator=(std::move(other));

  F_ = other.F_;
  other.F_ = nullptr;
  arm_mat_init_f32(&F_mat_, x_dim_, x_dim_, F_);

  B_ = other.B_;
  other.B_ = nullptr;
  arm_mat_init_f32(&B_mat_, x_dim_, u_dim_, B_);

  H_ = other.H_;
  other.H_ = nullptr;
  arm_mat_init_f32(&H_mat_, z_dim_, x_dim_, H_);

  Q_ = other.Q_;
  other.Q_ = nullptr;
  arm_mat_init_f32(&Q_mat_, x_dim_, x_dim_, Q_);

  R_ = other.R_;
  other.R_ = nullptr;
  arm_mat_init_f32(&R_mat_, z_dim_, z_dim_, R_);

  P_ = other.P_;
  other.P_ = nullptr;
  arm_mat_init_f32(&P_mat_, x_dim_, x_dim_, P_);

  return *this;
}

Kalman::~Kalman(void)
{
  Allocator<float>::deallocate(P_, x_dim_ * x_dim_);
  Allocator<float>::deallocate(R_, z_dim_ * z_dim_);
  Allocator<float>::deallocate(Q_, x_dim_ * x_dim_);
  Allocator<float>::deallocate(H_, z_dim_ * x_dim_);
  Allocator<float>::deallocate(B_, x_dim_ * u_dim_);
  Allocator<float>::deallocate(F_, x_dim_ * x_dim_);
}

void Kalman::calc(const float u[], const float z[])
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

void Kalman::setX0(const float x0[])
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(x0 != nullptr, "x0 must not be nullptr");
#pragma endregion

  memcpy(x_hat_, x0, x_dim_ * sizeof(float));
}

void Kalman::init(const Config& config_d)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(config_d.F != nullptr, "F must not be nullptr");
  HW_ASSERT(config_d.B != nullptr, "B must not be nullptr");
  HW_ASSERT(config_d.H != nullptr, "H must not be nullptr");
  HW_ASSERT(config_d.Q != nullptr, "Q must not be nullptr");
  HW_ASSERT(config_d.R != nullptr, "R must not be nullptr");
#pragma endregion

  if (F_ != nullptr) {
    Allocator<float>::deallocate(F_, x_dim_ * x_dim_);
  }

  if (B_ != nullptr) {
    Allocator<float>::deallocate(B_, x_dim_ * u_dim_);
  }

  if (H_ != nullptr) {
    Allocator<float>::deallocate(H_, z_dim_ * x_dim_);
  }

  if (Q_ != nullptr) {
    Allocator<float>::deallocate(Q_, x_dim_ * x_dim_);
  }

  if (R_ != nullptr) {
    Allocator<float>::deallocate(R_, z_dim_ * z_dim_);
  }

  if (P_ != nullptr) {
    Allocator<float>::deallocate(P_, x_dim_ * x_dim_);
  }

  Observer::init(config_d.x_dim, config_d.z_dim, config_d.u_dim);

  F_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  memcpy(F_, config_d.F, x_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&F_mat_, x_dim_, x_dim_, F_);

  B_ = Allocator<float>::allocate(x_dim_ * u_dim_);
  memcpy(B_, config_d.B, x_dim_ * u_dim_ * sizeof(float));
  arm_mat_init_f32(&B_mat_, x_dim_, u_dim_, B_);

  H_ = Allocator<float>::allocate(z_dim_ * x_dim_);
  memcpy(H_, config_d.H, z_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&H_mat_, z_dim_, x_dim_, H_);

  Q_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  memcpy(Q_, config_d.Q, x_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&Q_mat_, x_dim_, x_dim_, Q_);

  R_ = Allocator<float>::allocate(z_dim_ * z_dim_);
  memcpy(R_, config_d.R, z_dim_ * z_dim_ * sizeof(float));
  arm_mat_init_f32(&R_mat_, z_dim_, z_dim_, R_);

  P_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  if (config_d.P != nullptr) {
    memcpy(P_, config_d.P, x_dim_ * x_dim_ * sizeof(float));

  } else {
    memset(P_, 0, x_dim_ * x_dim_ * sizeof(float));
  }
  arm_mat_init_f32(&P_mat_, x_dim_, x_dim_, P_);

  if (config_d.x0 != nullptr) {
    setX0(config_d.x0);
  }
}

void Kalman::predict(const float u[], float x_hat_bar[],
                     arm_matrix_instance_f32& P_bar_mat) const
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(u != nullptr, "u must not be nullptr");
  HW_ASSERT(x_hat_bar != nullptr, "x_hat_bar must not be nullptr");
#pragma endregion

  /* x_hat_bar = F * x_hat_ + B * u */
  {
    float* tmp1 = Allocator<float>::allocate(x_dim_);
    float* tmp2 = Allocator<float>::allocate(x_dim_);
    arm_mat_vec_mult_f32(&F_mat_, x_hat_, tmp1);  // F * x_hat_
    arm_mat_vec_mult_f32(&B_mat_, u, tmp2);       // B * u
    arm_add_f32(tmp1, tmp2, x_hat_bar, x_dim_);   // F * x_hat_ + B * u

    Allocator<float>::deallocate(tmp2, x_dim_);
    Allocator<float>::deallocate(tmp1, x_dim_);
  }

  /* P_bar = F * P * F' + Q */
  {
    float* mat_tmp1 = Allocator<float>::allocate(x_dim_ * x_dim_);
    float* mat_tmp2 = Allocator<float>::allocate(x_dim_ * x_dim_);
    arm_matrix_instance_f32 mat_tmp_mat1, mat_tmp_mat2;
    arm_mat_init_f32(&mat_tmp_mat1, x_dim_, x_dim_, mat_tmp1);
    arm_mat_init_f32(&mat_tmp_mat2, x_dim_, x_dim_, mat_tmp2);

    arm_mat_trans_f32(&F_mat_, &mat_tmp_mat1);                // F'
    arm_mat_mult_f32(&P_mat_, &mat_tmp_mat1, &mat_tmp_mat2);  // P * F'
    arm_mat_mult_f32(&F_mat_, &mat_tmp_mat2, &mat_tmp_mat1);  // F * P * F'
    arm_mat_add_f32(&mat_tmp_mat1, &Q_mat_, &P_bar_mat);      // F * P * F' + Q

    Allocator<float>::deallocate(mat_tmp2, x_dim_ * x_dim_);
    Allocator<float>::deallocate(mat_tmp1, x_dim_ * x_dim_);
  }
}

void Kalman::update(const float z[], const float x_hat_bar[],
                    const arm_matrix_instance_f32& P_bar_mat)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(z != nullptr, "z must not be nullptr");
  HW_ASSERT(x_hat_bar != nullptr, "x_hat_bar must not be nullptr");
#pragma endregion

  float* K = Allocator<float>::allocate(x_dim_ * z_dim_);
  arm_matrix_instance_f32 K_mat;
  arm_mat_init_f32(&K_mat, x_dim_, z_dim_, K);

  /* K = P_bar * H' * (H * P_bar * H' + R)^(-1) */
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

    arm_mat_trans_f32(&H_mat_, &mat_tmp_mat1);                   // H'
    arm_mat_mult_f32(&P_bar_mat, &mat_tmp_mat1, &mat_tmp_mat2);  // P_bar * H'
    arm_mat_mult_f32(&H_mat_, &mat_tmp_mat2, &mat_tmp_mat3);     // H * P_bar * H'
    arm_mat_add_f32(&mat_tmp_mat3, &R_mat_, &mat_tmp_mat4);      // H * P_bar * H' + R
    arm_mat_inverse_f32(&mat_tmp_mat4, &mat_tmp_mat3);           // (H * P_bar * H' + R)^(-1)
    arm_mat_mult_f32(&mat_tmp_mat2, &mat_tmp_mat3, &K_mat);      // P_bar * H' * (H * P_bar * H' + R)^(-1)

    Allocator<float>::deallocate(mat_tmp4, z_dim_ * z_dim_);
    Allocator<float>::deallocate(mat_tmp3, z_dim_ * z_dim_);
    Allocator<float>::deallocate(mat_tmp2, x_dim_ * z_dim_);
    Allocator<float>::deallocate(mat_tmp1, x_dim_ * z_dim_);
  }

  /* x_hat_ = x_hat_bar + K * (z - H * x_hat_bar) */
  {
    float* tmp1 = Allocator<float>::allocate(z_dim_);
    float* tmp2 = Allocator<float>::allocate(z_dim_);
    float* tmp3 = Allocator<float>::allocate(x_dim_);
    arm_mat_vec_mult_f32(&H_mat_, x_hat_bar, tmp1);  // H * x_hat_bar
    arm_sub_f32(z, tmp1, tmp2, z_dim_);              // z - H * x_hat_bar
    arm_mat_vec_mult_f32(&K_mat, tmp2, tmp3);        // K * (z - H * x_hat_bar)
    arm_add_f32(x_hat_bar, tmp3, x_hat_, x_dim_);    // x_hat_bar + K * (z - H * x_hat_bar)

    Allocator<float>::deallocate(tmp3, x_dim_);
    Allocator<float>::deallocate(tmp2, z_dim_);
    Allocator<float>::deallocate(tmp1, z_dim_);
  }

  /* P = (I - K * H) * P_bar */
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

    arm_mat_mult_f32(&K_mat, &H_mat_, &mat_tmp_mat1);       // K * H
    arm_mat_sub_f32(&I_mat, &mat_tmp_mat1, &mat_tmp_mat2);  // I - K * H
    arm_mat_mult_f32(&mat_tmp_mat2, &P_bar_mat, &P_mat_);   // (I - K * H) * P_bar

    Allocator<float>::deallocate(I, x_dim_ * x_dim_);
    Allocator<float>::deallocate(mat_tmp2, x_dim_ * x_dim_);
    Allocator<float>::deallocate(mat_tmp1, x_dim_ * x_dim_);
  }

  Allocator<float>::deallocate(K, x_dim_ * z_dim_);
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace observer
}  // namespace hello_world
