/**
 *******************************************************************************
 * @file      : luenberger.cpp
 * @brief     : 龙伯格观测器
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
#include "luenberger.hpp"

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

Luenberger::Luenberger(const Config& config_d)
    : Observer(config_d.x_dim, config_d.z_dim, config_d.u_dim)
{
/* 变量检查 */
#pragma region
  HW_ASSERT(config_d.F != nullptr, "F must not be nullptr");
  HW_ASSERT(config_d.B != nullptr, "B must not be nullptr");
  HW_ASSERT(config_d.H != nullptr, "H must not be nullptr");
  HW_ASSERT(config_d.L != nullptr, "L must not be nullptr");
#pragma endregion

  Fd_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  memcpy(Fd_, config_d.F, x_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&Fd_mat_, x_dim_, x_dim_, Fd_);

  Bd_ = Allocator<float>::allocate(x_dim_ * u_dim_);
  memcpy(Bd_, config_d.B, x_dim_ * u_dim_ * sizeof(float));
  arm_mat_init_f32(&Bd_mat_, x_dim_, u_dim_, Bd_);

  Hd_ = Allocator<float>::allocate(z_dim_ * x_dim_);
  memcpy(Hd_, config_d.H, z_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&Hd_mat_, z_dim_, x_dim_, Hd_);

  Ld_ = Allocator<float>::allocate(x_dim_ * z_dim_);
  memcpy(Ld_, config_d.L, x_dim_ * z_dim_ * sizeof(float));
  arm_mat_init_f32(&Ld_mat_, x_dim_, z_dim_, Ld_);

  if (config_d.x0 != nullptr) {
    setX0(config_d.x0);
  }
}

Luenberger::Luenberger(const Config& config_c, float samp_period)
    : Observer(config_c.x_dim, config_c.z_dim, config_c.u_dim)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(config_c.F != nullptr, "F must not be nullptr");
  HW_ASSERT(config_c.B != nullptr, "B must not be nullptr");
  HW_ASSERT(config_c.H != nullptr, "H must not be nullptr");
  HW_ASSERT(config_c.L != nullptr, "L must not be nullptr");
  HW_ASSERT(samp_period > 0, "samp_period must be greater than 0");
#pragma endregion

  /* 系统矩阵离散化 */
  Fd_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  {
    arm_matrix_instance_f32 Fc_mat;
    arm_mat_init_f32(&Fc_mat, x_dim_, x_dim_, config_c.F);

    float* I = Allocator<float>::allocate(x_dim_ * x_dim_);
    memset(I, 0, x_dim_ * x_dim_ * sizeof(float));
    for (size_t i = 0; i < x_dim_; i++) {
      I[i * x_dim_ + i] = 1.0f;
    }
    arm_matrix_instance_f32 I_mat;
    arm_mat_init_f32(&I_mat, x_dim_, x_dim_, I);

    float* tmp = Allocator<float>::allocate(x_dim_ * x_dim_);
    arm_matrix_instance_f32 tmp_mat;
    arm_mat_init_f32(&tmp_mat, x_dim_, x_dim_, tmp);
    arm_mat_init_f32(&Fd_mat_, x_dim_, x_dim_, Fd_);
    arm_mat_scale_f32(&Fc_mat, samp_period, &tmp_mat);
    arm_mat_add_f32(&tmp_mat, &I_mat, &Fd_mat_);

    Allocator<float>::deallocate(tmp, x_dim_ * x_dim_);
    Allocator<float>::deallocate(I, x_dim_ * x_dim_);
  }

  /* 控制矩阵离散化 */
  Bd_ = Allocator<float>::allocate(x_dim_ * u_dim_);
  {
    arm_matrix_instance_f32 Bc_mat;
    arm_mat_init_f32(&Bc_mat, x_dim_, u_dim_, config_c.B);

    arm_mat_init_f32(&Bd_mat_, x_dim_, u_dim_, Bd_);
    arm_mat_scale_f32(&Bc_mat, samp_period, &Bd_mat_);
  }

  /* 观测矩阵离散化 */
  Hd_ = Allocator<float>::allocate(z_dim_ * x_dim_);
  memcpy(Hd_, config_c.H, z_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&Hd_mat_, z_dim_, x_dim_, Hd_);

  /* 龙伯格增益矩阵离散化 */
  Ld_ = Allocator<float>::allocate(x_dim_ * z_dim_);
  {
    arm_matrix_instance_f32 Lc_mat;
    arm_mat_init_f32(&Lc_mat, x_dim_, z_dim_, config_c.L);

    arm_mat_init_f32(&Ld_mat_, x_dim_, z_dim_, Ld_);
    arm_mat_scale_f32(&Lc_mat, samp_period, &Ld_mat_);
  }

  if (config_c.x0 != nullptr) {
    setX0(config_c.x0);
  }
}

Luenberger::Luenberger(const Luenberger& other)
    : Observer(other)
{
  Fd_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  memcpy(Fd_, other.Fd_, x_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&Fd_mat_, x_dim_, x_dim_, Fd_);

  Bd_ = Allocator<float>::allocate(x_dim_ * u_dim_);
  memcpy(Bd_, other.Bd_, x_dim_ * u_dim_ * sizeof(float));
  arm_mat_init_f32(&Bd_mat_, x_dim_, u_dim_, Bd_);

  Hd_ = Allocator<float>::allocate(z_dim_ * x_dim_);
  memcpy(Hd_, other.Hd_, z_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&Hd_mat_, z_dim_, x_dim_, Hd_);

  Ld_ = Allocator<float>::allocate(x_dim_ * z_dim_);
  memcpy(Ld_, other.Ld_, x_dim_ * z_dim_ * sizeof(float));
  arm_mat_init_f32(&Ld_mat_, x_dim_, z_dim_, Ld_);
}

Luenberger& Luenberger::operator=(const Luenberger& other)
{
  if (this == &other) {
    return *this;
  }

  if (Fd_ != nullptr) {
    Allocator<float>::deallocate(Fd_, x_dim_ * x_dim_);
  }

  if (Bd_ != nullptr) {
    Allocator<float>::deallocate(Bd_, x_dim_ * u_dim_);
  }

  if (Hd_ != nullptr) {
    Allocator<float>::deallocate(Hd_, z_dim_ * x_dim_);
  }

  if (Ld_ != nullptr) {
    Allocator<float>::deallocate(Ld_, x_dim_ * z_dim_);
  }

  Observer::operator=(other);

  Fd_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  memcpy(Fd_, other.Fd_, x_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&Fd_mat_, x_dim_, x_dim_, Fd_);

  Bd_ = Allocator<float>::allocate(x_dim_ * u_dim_);
  memcpy(Bd_, other.Bd_, x_dim_ * u_dim_ * sizeof(float));
  arm_mat_init_f32(&Bd_mat_, x_dim_, u_dim_, Bd_);

  Hd_ = Allocator<float>::allocate(z_dim_ * x_dim_);
  memcpy(Hd_, other.Hd_, z_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&Hd_mat_, z_dim_, x_dim_, Hd_);

  Ld_ = Allocator<float>::allocate(x_dim_ * z_dim_);
  memcpy(Ld_, other.Ld_, x_dim_ * z_dim_ * sizeof(float));
  arm_mat_init_f32(&Ld_mat_, x_dim_, z_dim_, Ld_);

  return *this;
}

Luenberger::Luenberger(Luenberger&& other)
    : Observer(std::move(other))
{
  Fd_ = other.Fd_;
  other.Fd_ = nullptr;
  arm_mat_init_f32(&Fd_mat_, x_dim_, x_dim_, Fd_);

  Bd_ = other.Bd_;
  other.Bd_ = nullptr;
  arm_mat_init_f32(&Bd_mat_, x_dim_, u_dim_, Bd_);

  Hd_ = other.Hd_;
  other.Hd_ = nullptr;
  arm_mat_init_f32(&Hd_mat_, z_dim_, x_dim_, Hd_);

  Ld_ = other.Ld_;
  other.Ld_ = nullptr;
  arm_mat_init_f32(&Ld_mat_, x_dim_, z_dim_, Ld_);
}

Luenberger& Luenberger::operator=(Luenberger&& other)
{
  if (this == &other) {
    return *this;
  }

  if (Fd_ != nullptr) {
    Allocator<float>::deallocate(Fd_, x_dim_ * x_dim_);
  }

  if (Bd_ != nullptr) {
    Allocator<float>::deallocate(Bd_, x_dim_ * u_dim_);
  }

  if (Hd_ != nullptr) {
    Allocator<float>::deallocate(Hd_, z_dim_ * x_dim_);
  }

  if (Ld_ != nullptr) {
    Allocator<float>::deallocate(Ld_, x_dim_ * z_dim_);
  }

  Observer::operator=(std::move(other));

  Fd_ = other.Fd_;
  other.Fd_ = nullptr;
  arm_mat_init_f32(&Fd_mat_, x_dim_, x_dim_, Fd_);

  Bd_ = other.Bd_;
  other.Bd_ = nullptr;
  arm_mat_init_f32(&Bd_mat_, x_dim_, u_dim_, Bd_);

  Hd_ = other.Hd_;
  other.Hd_ = nullptr;
  arm_mat_init_f32(&Hd_mat_, z_dim_, x_dim_, Hd_);

  Ld_ = other.Ld_;
  other.Ld_ = nullptr;
  arm_mat_init_f32(&Ld_mat_, x_dim_, z_dim_, Ld_);

  return *this;
}

Luenberger::~Luenberger(void)
{
  Allocator<float>::deallocate(Ld_, x_dim_ * z_dim_);
  Allocator<float>::deallocate(Hd_, z_dim_ * x_dim_);
  Allocator<float>::deallocate(Bd_, x_dim_ * u_dim_);
  Allocator<float>::deallocate(Fd_, x_dim_ * x_dim_);
}

void Luenberger::calc(const float u[], const float z[])
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(u != nullptr, "u must not be nullptr");
  HW_ASSERT(z != nullptr, "z must not be nullptr");
#pragma endregion

  float* x_hat_bar = Allocator<float>::allocate(x_dim_);
  float* tmp1 = Allocator<float>::allocate(x_dim_);
  float* tmp2 = Allocator<float>::allocate(x_dim_);

  /* 预测 */
  /* x_hat_bar = Fd * x_hat_ + Bd * u */
  arm_mat_vec_mult_f32(&Fd_mat_, x_hat_, tmp1);
  arm_mat_vec_mult_f32(&Bd_mat_, u, tmp2);
  arm_add_f32(tmp1, tmp2, x_hat_bar, x_dim_);

  /* 校正 */
  /* x_hat_ = x_hat_bar + Ld * (z - Hd * x_hat_bar) */
  float* z_hat_bar = Allocator<float>::allocate(z_dim_);
  float* err = Allocator<float>::allocate(z_dim_);
  arm_mat_vec_mult_f32(&Hd_mat_, x_hat_bar, z_hat_bar);
  arm_sub_f32(z, z_hat_bar, err, z_dim_);
  arm_mat_vec_mult_f32(&Ld_mat_, err, tmp1);
  arm_add_f32(x_hat_bar, tmp1, x_hat_, x_dim_);

  Allocator<float>::deallocate(err, z_dim_);
  Allocator<float>::deallocate(z_hat_bar, z_dim_);
  Allocator<float>::deallocate(tmp2, x_dim_);
  Allocator<float>::deallocate(tmp1, x_dim_);
  Allocator<float>::deallocate(x_hat_bar, x_dim_);
}

void Luenberger::setX0(const float x0[])
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(x0 != nullptr, "x must not be nullptr");
#pragma endregion

  memcpy(x_hat_, x0, x_dim_ * sizeof(float));
}

void Luenberger::init(const Config& config_d)
{
/* 变量检查 */
#pragma region
  HW_ASSERT(config_d.F != nullptr, "F must not be nullptr");
  HW_ASSERT(config_d.B != nullptr, "B must not be nullptr");
  HW_ASSERT(config_d.H != nullptr, "H must not be nullptr");
  HW_ASSERT(config_d.L != nullptr, "L must not be nullptr");
#pragma endregion

  if (Fd_ != nullptr) {
    Allocator<float>::deallocate(Fd_, x_dim_ * x_dim_);
  }

  if (Bd_ != nullptr) {
    Allocator<float>::deallocate(Bd_, x_dim_ * u_dim_);
  }

  if (Hd_ != nullptr) {
    Allocator<float>::deallocate(Hd_, z_dim_ * x_dim_);
  }

  if (Ld_ != nullptr) {
    Allocator<float>::deallocate(Ld_, x_dim_ * z_dim_);
  }

  Observer::init(config_d.x_dim, config_d.z_dim, config_d.u_dim);

  Fd_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  memcpy(Fd_, config_d.F, x_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&Fd_mat_, x_dim_, x_dim_, Fd_);

  Bd_ = Allocator<float>::allocate(x_dim_ * u_dim_);
  memcpy(Bd_, config_d.B, x_dim_ * u_dim_ * sizeof(float));
  arm_mat_init_f32(&Bd_mat_, x_dim_, u_dim_, Bd_);

  Hd_ = Allocator<float>::allocate(z_dim_ * x_dim_);
  memcpy(Hd_, config_d.H, z_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&Hd_mat_, z_dim_, x_dim_, Hd_);

  Ld_ = Allocator<float>::allocate(x_dim_ * z_dim_);
  memcpy(Ld_, config_d.L, x_dim_ * z_dim_ * sizeof(float));
  arm_mat_init_f32(&Ld_mat_, x_dim_, z_dim_, Ld_);

  if (config_d.x0 != nullptr) {
    setX0(config_d.x0);
  }
}

void Luenberger::init(const Config& config_c, float samp_period)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(config_c.F != nullptr, "F must not be nullptr");
  HW_ASSERT(config_c.B != nullptr, "B must not be nullptr");
  HW_ASSERT(config_c.H != nullptr, "H must not be nullptr");
  HW_ASSERT(config_c.L != nullptr, "L must not be nullptr");
  HW_ASSERT(samp_period > 0, "samp_period must be greater than 0");
#pragma endregion

  if (Fd_ != nullptr) {
    Allocator<float>::deallocate(Fd_, x_dim_ * x_dim_);
  }

  if (Bd_ != nullptr) {
    Allocator<float>::deallocate(Bd_, x_dim_ * u_dim_);
  }

  if (Hd_ != nullptr) {
    Allocator<float>::deallocate(Hd_, z_dim_ * x_dim_);
  }

  if (Ld_ != nullptr) {
    Allocator<float>::deallocate(Ld_, x_dim_ * z_dim_);
  }

  Observer::init(config_c.x_dim, config_c.z_dim, config_c.u_dim);

  /* 系统矩阵离散化 */
  Fd_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  {
    arm_matrix_instance_f32 Fc_mat;
    arm_mat_init_f32(&Fc_mat, x_dim_, x_dim_, config_c.F);

    float* I = Allocator<float>::allocate(x_dim_ * x_dim_);
    memset(I, 0, x_dim_ * x_dim_ * sizeof(float));
    for (size_t i = 0; i < x_dim_; i++) {
      I[i * x_dim_ + i] = 1.0f;
    }
    arm_matrix_instance_f32 I_mat;
    arm_mat_init_f32(&I_mat, x_dim_, x_dim_, I);

    float* tmp = Allocator<float>::allocate(x_dim_ * x_dim_);
    arm_matrix_instance_f32 tmp_mat;
    arm_mat_init_f32(&tmp_mat, x_dim_, x_dim_, tmp);
    arm_mat_init_f32(&Fd_mat_, x_dim_, x_dim_, Fd_);
    arm_mat_scale_f32(&Fc_mat, samp_period, &tmp_mat);
    arm_mat_add_f32(&tmp_mat, &I_mat, &Fd_mat_);

    Allocator<float>::deallocate(tmp, x_dim_ * x_dim_);
    Allocator<float>::deallocate(I, x_dim_ * x_dim_);
  }

  /* 控制矩阵离散化 */
  Bd_ = Allocator<float>::allocate(x_dim_ * u_dim_);
  {
    arm_matrix_instance_f32 Bc_mat;
    arm_mat_init_f32(&Bc_mat, x_dim_, u_dim_, config_c.B);

    arm_mat_init_f32(&Bd_mat_, x_dim_, u_dim_, Bd_);
    arm_mat_scale_f32(&Bc_mat, samp_period, &Bd_mat_);
  }

  /* 观测矩阵离散化 */
  Hd_ = Allocator<float>::allocate(z_dim_ * x_dim_);
  memcpy(Hd_, config_c.H, z_dim_ * x_dim_ * sizeof(float));
  arm_mat_init_f32(&Hd_mat_, z_dim_, x_dim_, Hd_);

  /* 龙伯格增益矩阵离散化 */
  Ld_ = Allocator<float>::allocate(x_dim_ * z_dim_);
  {
    arm_matrix_instance_f32 Lc_mat;
    arm_mat_init_f32(&Lc_mat, x_dim_, z_dim_, config_c.L);

    arm_mat_init_f32(&Ld_mat_, x_dim_, z_dim_, Ld_);
    arm_mat_scale_f32(&Lc_mat, samp_period, &Ld_mat_);
  }

  if (config_c.x0 != nullptr) {
    setX0(config_c.x0);
  }
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace observer
}  // namespace hello_world
