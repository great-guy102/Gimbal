/**
 *******************************************************************************
 * @file      : ukf.cpp
 * @brief     : 无迹卡尔曼滤波器
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-12-29      Caikunzhen      1. 完成初版编写（未测试）
 *  V1.0.0      2024-01-03      Caikunzhen      1. 完成测试
 *  V1.1.0      2004-07-13      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  1. 该滤波器有发散的可能，使用时请谨慎
 *  2. 教程详见：https://blog.csdn.net/qq_41011336/article/details/84401691
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ukf.hpp"

#include <cstring>

#include "arm_math.h"
#include "assert.hpp"

namespace hello_world
{
namespace observer
{
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

HW_OPTIMIZE_O2_START

/**
 * @brief       Cholesky分解
 * @param        n: 矩阵维度
 * @param        A: 待分解矩阵，可只含有下三角部分，返回分解后的下三角部分，大小为
 *               (n, n)，按行存储
 * @retval       None
 * @note        None
 */
static void CholeskyDecomposition(size_t n, float* A);
/* Exported function definitions ---------------------------------------------*/

Ukf::Ukf(const Config& config)
    : Observer(config.x_dim, config.z_dim, config.u_dim),
      Q_dim_(config.Q_dim),
      R_dim_(config.R_dim),
      Na_(x_dim_ + Q_dim_ + R_dim_),
      alpha_(config.alpha),
      beta_(config.beta),
      kappa_(config.kappa),
      lamda_(alpha_ * alpha_ * (Na_ + kappa_) - Na_),
      f_(config.f),
      h_(config.h)
{
  /* 变量检查 */
#pragma reginon
  HW_ASSERT(config.Q_dim > 0, "Q_dim must be greater than 0");
  HW_ASSERT(config.R_dim > 0, "R_dim must be greater than 0");
  HW_ASSERT(config.alpha >= 0 && config.alpha <= 1,
            "alpha must be in [0, 1]");
  HW_ASSERT(config.beta >= 0, "beta must be greater than 0");
  HW_ASSERT(config.kappa >= 0, "kappa must be greater than 0");
  HW_ASSERT(config.f != nullptr, "f must not be nullptr");
  HW_ASSERT(config.h != nullptr, "h must not be nullptr");
#pragma endreginon

  Q_ = Allocator<float>::allocate(Q_dim_ * Q_dim_);
  memcpy(Q_, config.Q, sizeof(float) * Q_dim_ * Q_dim_);
  R_ = Allocator<float>::allocate(R_dim_ * R_dim_);
  memcpy(R_, config.R, sizeof(float) * R_dim_ * R_dim_);

  /* 计算增广 sigma 点中噪声的部分 */
  float gamma = sqrtf(Na_ + lamda_);
  /* gamma * sqrt(Q) 的列向量 */
  X_w_ = Allocator<float>::allocate(2 * Q_dim_ * Q_dim_);
  CholeskyDecomposition(Q_dim_, Q_);
  for (size_t i = 0; i < Q_dim_; i++) {
    for (size_t j = 0; j < Q_dim_; j++) {
      X_w_[i * Q_dim_ + j] = gamma * Q_[j * Q_dim_ + i];
      X_w_[(i + Q_dim_) * Q_dim_ + j] = -X_w_[i * Q_dim_ + j];
    }
  }

  /* gamma * sqrt(R) 的列向量 */
  X_v_ = Allocator<float>::allocate(2 * R_dim_ * R_dim_);
  CholeskyDecomposition(R_dim_, R_);
  for (size_t i = 0; i < R_dim_; i++) {
    for (size_t j = 0; j < R_dim_; j++) {
      X_v_[i * R_dim_ + j] = gamma * R_[j * R_dim_ + i];
      X_v_[(i + R_dim_) * R_dim_ + j] = -X_v_[i * R_dim_ + j];
    }
  }

  w_mean_ = Allocator<float>::allocate(Q_dim_);
  memset(w_mean_, 0, sizeof(float) * Q_dim_);
  v_mean_ = Allocator<float>::allocate(R_dim_);
  memset(v_mean_, 0, sizeof(float) * R_dim_);

  P_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  memcpy(P_, config.P, sizeof(float) * x_dim_ * x_dim_);

  if (config.x0 != nullptr) {
    setX0(config.x0);
  }
}

Ukf::Ukf(const Ukf& other)
    : Observer(other),
      Q_dim_(other.Q_dim_),
      R_dim_(other.R_dim_),
      Na_(other.Na_),
      alpha_(other.alpha_),
      beta_(other.beta_),
      kappa_(other.kappa_),
      lamda_(other.lamda_),
      f_(other.f_),
      h_(other.h_)
{
  Q_ = Allocator<float>::allocate(Q_dim_ * Q_dim_);
  memcpy(Q_, other.Q_, sizeof(float) * Q_dim_ * Q_dim_);
  R_ = Allocator<float>::allocate(R_dim_ * R_dim_);
  memcpy(R_, other.R_, sizeof(float) * R_dim_ * R_dim_);

  X_w_ = Allocator<float>::allocate(2 * Q_dim_ * Q_dim_);
  memcpy(X_w_, other.X_w_, sizeof(float) * 2 * Q_dim_ * Q_dim_);
  X_v_ = Allocator<float>::allocate(2 * R_dim_ * R_dim_);
  memcpy(X_v_, other.X_v_, sizeof(float) * 2 * R_dim_ * R_dim_);

  w_mean_ = Allocator<float>::allocate(Q_dim_);
  memcpy(w_mean_, other.w_mean_, sizeof(float) * Q_dim_);
  v_mean_ = Allocator<float>::allocate(R_dim_);
  memcpy(v_mean_, other.v_mean_, sizeof(float) * R_dim_);

  P_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  memcpy(P_, other.P_, sizeof(float) * x_dim_ * x_dim_);

  if (other.x_hat_ != nullptr) {
    setX0(other.x_hat_);
  }
}

Ukf& Ukf::operator=(const Ukf& other)
{
  if (this == &other) {
    return *this;
  }

  if (Q_ != nullptr) {
    Allocator<float>::deallocate(Q_, Q_dim_ * Q_dim_);
  }

  if (R_ != nullptr) {
    Allocator<float>::deallocate(R_, R_dim_ * R_dim_);
  }

  if (P_ != nullptr) {
    Allocator<float>::deallocate(P_, x_dim_ * x_dim_);
  }

  if (X_w_ != nullptr) {
    Allocator<float>::deallocate(X_w_, 2 * Q_dim_ * Q_dim_);
  }

  if (X_v_ != nullptr) {
    Allocator<float>::deallocate(X_v_, 2 * R_dim_ * R_dim_);
  }

  if (w_mean_ != nullptr) {
    Allocator<float>::deallocate(w_mean_, Q_dim_);
  }

  if (v_mean_ != nullptr) {
    Allocator<float>::deallocate(v_mean_, R_dim_);
  }

  Observer::operator=(other);

  Q_dim_ = other.Q_dim_;
  R_dim_ = other.R_dim_;
  Na_ = other.Na_;
  alpha_ = other.alpha_;
  beta_ = other.beta_;
  kappa_ = other.kappa_;
  lamda_ = other.lamda_;
  f_ = other.f_;
  h_ = other.h_;

  Q_ = Allocator<float>::allocate(Q_dim_ * Q_dim_);
  memcpy(Q_, other.Q_, sizeof(float) * Q_dim_ * Q_dim_);
  R_ = Allocator<float>::allocate(R_dim_ * R_dim_);
  memcpy(R_, other.R_, sizeof(float) * R_dim_ * R_dim_);

  X_w_ = Allocator<float>::allocate(2 * Q_dim_ * Q_dim_);
  memcpy(X_w_, other.X_w_, sizeof(float) * 2 * Q_dim_ * Q_dim_);
  X_v_ = Allocator<float>::allocate(2 * R_dim_ * R_dim_);
  memcpy(X_v_, other.X_v_, sizeof(float) * 2 * R_dim_ * R_dim_);

  w_mean_ = Allocator<float>::allocate(Q_dim_);
  memcpy(w_mean_, other.w_mean_, sizeof(float) * Q_dim_);
  v_mean_ = Allocator<float>::allocate(R_dim_);
  memcpy(v_mean_, other.v_mean_, sizeof(float) * R_dim_);

  P_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  memcpy(P_, other.P_, sizeof(float) * x_dim_ * x_dim_);

  if (other.x_hat_ != nullptr) {
    setX0(other.x_hat_);
  }

  return *this;
}

Ukf::Ukf(Ukf&& other)
    : Observer(std::move(other)),
      Q_dim_(other.Q_dim_),
      R_dim_(other.R_dim_),
      Na_(other.Na_),
      alpha_(other.alpha_),
      beta_(other.beta_),
      kappa_(other.kappa_),
      lamda_(other.lamda_),
      f_(other.f_),
      h_(other.h_)
{
  Q_ = other.Q_;
  other.Q_ = nullptr;
  R_ = other.R_;
  other.R_ = nullptr;
  P_ = other.P_;
  other.P_ = nullptr;
  X_w_ = other.X_w_;
  other.X_w_ = nullptr;
  X_v_ = other.X_v_;
  other.X_v_ = nullptr;
  w_mean_ = other.w_mean_;
  other.w_mean_ = nullptr;
  v_mean_ = other.v_mean_;
  other.v_mean_ = nullptr;
}

Ukf& Ukf::operator=(Ukf&& other)
{
  if (this == &other) {
    return *this;
  }

  if (Q_ != nullptr) {
    Allocator<float>::deallocate(Q_, Q_dim_ * Q_dim_);
  }

  if (R_ != nullptr) {
    Allocator<float>::deallocate(R_, R_dim_ * R_dim_);
  }

  if (P_ != nullptr) {
    Allocator<float>::deallocate(P_, x_dim_ * x_dim_);
  }

  if (X_w_ != nullptr) {
    Allocator<float>::deallocate(X_w_, 2 * Q_dim_ * Q_dim_);
  }

  if (X_v_ != nullptr) {
    Allocator<float>::deallocate(X_v_, 2 * R_dim_ * R_dim_);
  }

  if (w_mean_ != nullptr) {
    Allocator<float>::deallocate(w_mean_, Q_dim_);
  }

  if (v_mean_ != nullptr) {
    Allocator<float>::deallocate(v_mean_, R_dim_);
  }

  Observer::operator=(std::move(other));

  Q_dim_ = other.Q_dim_;
  R_dim_ = other.R_dim_;
  Na_ = other.Na_;
  alpha_ = other.alpha_;
  beta_ = other.beta_;
  kappa_ = other.kappa_;
  lamda_ = other.lamda_;
  f_ = other.f_;
  h_ = other.h_;

  Q_ = other.Q_;
  other.Q_ = nullptr;
  R_ = other.R_;
  other.R_ = nullptr;
  P_ = other.P_;
  other.P_ = nullptr;
  X_w_ = other.X_w_;
  other.X_w_ = nullptr;
  X_v_ = other.X_v_;
  other.X_v_ = nullptr;
  w_mean_ = other.w_mean_;
  other.w_mean_ = nullptr;
  v_mean_ = other.v_mean_;
  other.v_mean_ = nullptr;

  return *this;
}

Ukf::~Ukf(void)
{
  Allocator<float>::deallocate(Q_, Q_dim_ * Q_dim_);

  Allocator<float>::deallocate(v_mean_, R_dim_);
  Allocator<float>::deallocate(w_mean_, Q_dim_);

  Allocator<float>::deallocate(X_v_, 2 * R_dim_ * R_dim_);
  Allocator<float>::deallocate(X_w_, 2 * Q_dim_ * Q_dim_);

  Allocator<float>::deallocate(R_, R_dim_ * R_dim_);
  Allocator<float>::deallocate(Q_, Q_dim_ * Q_dim_);
}

void Ukf::calc(const float u[], const float z[])
{
  /* 变量检查 */
#pragma reginon
  HW_ASSERT(u != nullptr, "u must not be nullptr");
  HW_ASSERT(z != nullptr, "z must not be nullptr");
#pragma endreginon

  float* x_hat_bar = Allocator<float>::allocate(x_dim_);
  float* P_bar = Allocator<float>::allocate(x_dim_ * x_dim_);
  float* X_x_bar =
      Allocator<float>::allocate((2 * (x_dim_ + Q_dim_) + 1) * x_dim_);

  predict(u, x_hat_bar, P_bar, X_x_bar);

  update(z, x_hat_bar, P_bar, X_x_bar);

  Allocator<float>::deallocate(X_x_bar, (2 * (x_dim_ + Q_dim_) + 1) * x_dim_);
  Allocator<float>::deallocate(P_bar, x_dim_ * x_dim_);
  Allocator<float>::deallocate(x_hat_bar, x_dim_);
}

void Ukf::setX0(const float x0[])
{
  /* 变量检查 */
#pragma reginon
  HW_ASSERT(x0 != nullptr, "x must not be nullptr");
#pragma endreginon

  memcpy(x_hat_, x0, sizeof(float) * x_dim_);
}

void Ukf::init(const Config& config)
{
  /* 变量检查 */
#pragma reginon
  HW_ASSERT(config.Q_dim > 0, "Q_dim must be greater than 0");
  HW_ASSERT(config.R_dim > 0, "R_dim must be greater than 0");
  HW_ASSERT(config.alpha >= 0 && config.alpha <= 1,
            "alpha must be in [0, 1]");
  HW_ASSERT(config.beta >= 0, "beta must be greater than 0");
  HW_ASSERT(config.kappa >= 0, "kappa must be greater than 0");
  HW_ASSERT(config.f != nullptr, "f must not be nullptr");
  HW_ASSERT(config.h != nullptr, "h must not be nullptr");
#pragma endreginon

  if (Q_ != nullptr) {
    Allocator<float>::deallocate(Q_, Q_dim_ * Q_dim_);
  }

  if (R_ != nullptr) {
    Allocator<float>::deallocate(R_, R_dim_ * R_dim_);
  }

  if (P_ != nullptr) {
    Allocator<float>::deallocate(P_, x_dim_ * x_dim_);
  }

  if (X_w_ != nullptr) {
    Allocator<float>::deallocate(X_w_, 2 * Q_dim_ * Q_dim_);
  }

  if (X_v_ != nullptr) {
    Allocator<float>::deallocate(X_v_, 2 * R_dim_ * R_dim_);
  }

  if (w_mean_ != nullptr) {
    Allocator<float>::deallocate(w_mean_, Q_dim_);
  }

  if (v_mean_ != nullptr) {
    Allocator<float>::deallocate(v_mean_, R_dim_);
  }

  Observer::init(config.x_dim, config.z_dim, config.u_dim);

  Q_dim_ = config.Q_dim;
  R_dim_ = config.R_dim;
  Na_ = x_dim_ + Q_dim_ + R_dim_;
  alpha_ = config.alpha;
  beta_ = config.beta;
  kappa_ = config.kappa;
  lamda_ = alpha_ * alpha_ * (Na_ + kappa_) - Na_;
  f_ = config.f;
  h_ = config.h;

  Q_ = Allocator<float>::allocate(Q_dim_ * Q_dim_);
  memcpy(Q_, config.Q, sizeof(float) * Q_dim_ * Q_dim_);
  R_ = Allocator<float>::allocate(R_dim_ * R_dim_);
  memcpy(R_, config.R, sizeof(float) * R_dim_ * R_dim_);

  /* 计算增广 sigma 点中噪声的部分 */
  float gamma = sqrtf(Na_ + lamda_);
  /* gamma * sqrt(Q) 的列向量 */
  X_w_ = Allocator<float>::allocate(2 * Q_dim_ * Q_dim_);
  CholeskyDecomposition(Q_dim_, Q_);
  for (size_t i = 0; i < Q_dim_; i++) {
    for (size_t j = 0; j < Q_dim_; j++) {
      X_w_[i * Q_dim_ + j] = gamma * Q_[j * Q_dim_ + i];
      X_w_[(i + Q_dim_) * Q_dim_ + j] = -X_w_[i * Q_dim_ + j];
    }
  }

  /* gamma * sqrt(R) 的列向量 */
  X_v_ = Allocator<float>::allocate(2 * R_dim_ * R_dim_);
  CholeskyDecomposition(R_dim_, R_);
  for (size_t i = 0; i < R_dim_; i++) {
    for (size_t j = 0; j < R_dim_; j++) {
      X_v_[i * R_dim_ + j] = gamma * R_[j * R_dim_ + i];
      X_v_[(i + R_dim_) * R_dim_ + j] = -X_v_[i * R_dim_ + j];
    }
  }

  w_mean_ = Allocator<float>::allocate(Q_dim_);
  memset(w_mean_, 0, sizeof(float) * Q_dim_);
  v_mean_ = Allocator<float>::allocate(R_dim_);
  memset(v_mean_, 0, sizeof(float) * R_dim_);

  P_ = Allocator<float>::allocate(x_dim_ * x_dim_);
  memcpy(P_, config.P, sizeof(float) * x_dim_ * x_dim_);

  if (config.x0 != nullptr) {
    setX0(config.x0);
  }
}

void Ukf::getSigmaPnts(const float x[], const float* P, size_t n, float scale,
                       float* X) const
{
  /* 变量检查 */
#pragma reginon
  HW_ASSERT(x != nullptr, "x must not be nullptr");
  HW_ASSERT(P != nullptr, "P must not be nullptr");
  HW_ASSERT(X != nullptr, "X must not be nullptr");
#pragma endreginon

  float* L = Allocator<float>::allocate(n * n);
  /* 只获取下三角部分 */
  for (size_t i = 0; i < n; i++) {
    for (size_t j = 0; j <= i; j++) {
      L[i * n + j] = scale * P[i * n + j];
    }
  }
  CholeskyDecomposition(n, L);

  memcpy(X, x, sizeof(float) * n);                // i=0
  for (size_t i = 0; i < n; i++) {                // i=1~n，n+1~2n
    for (size_t j = 0; j < n; j++) {
      X[(i + 1) * n + j] = x[j] + L[j * n + i];
      X[(i + 1 + n) * n + j] = x[j] - L[j * n + i];
    }
  }

  Allocator<float>::deallocate(L, n * n);
}

void Ukf::predict(const float u[], float x_hat_bar[], float* P_bar,
                  float* X_x_bar) const
{
  /* 变量检查 */
#pragma reginon
  HW_ASSERT(u != nullptr, "u must not be nullptr");
  HW_ASSERT(x_hat_bar != nullptr, "x_hat_bar must not be nullptr");
  HW_ASSERT(P_bar != nullptr, "P_bar must not be nullptr");
  HW_ASSERT(X_x_bar != nullptr, "X_x_bar must not be nullptr");
#pragma endreginon

  const size_t kXxBarLsLen = 2 * (x_dim_ + Q_dim_) + 1;
  const size_t kXxLsLen = 2 * x_dim_ + 1;

  /* 含非线性噪声影响的下一时刻sigma点的预测值 */
  float(*X_x_bar_ls)[x_dim_] = reinterpret_cast<float(*)[x_dim_]>(X_x_bar);
  {
    float* X_x_ls_data = Allocator<float>::allocate(kXxLsLen * x_dim_);
    float(*X_x_ls)[x_dim_] = reinterpret_cast<float(*)[x_dim_]>(X_x_ls_data);
    getSigmaPnts(x_hat_, P_, x_dim_, Na_ + lamda_, X_x_ls_data);

    /* 平均值权重 */
    const float kWm = 1 / (2 * (lamda_ + Na_));
    const float kWm0 = lamda_ / (lamda_ + Na_) + R_dim_ / (lamda_ + Na_);
    memset(x_hat_bar, 0, sizeof(float) * x_dim_);

    float w_m = kWm0;
    for (size_t i = 0; i < kXxBarLsLen; i++) {
      if (i < kXxLsLen) {
        f_(X_x_ls[i], u, w_mean_, X_x_bar_ls[i]);
      } else {
        /* 噪声部分 */
        f_(X_x_ls[0], u, X_w_ + (i - kXxLsLen) * Q_dim_,
           X_x_bar_ls[i]);
      }

      for (size_t j = 0; j < x_dim_; j++) {
        x_hat_bar[j] += w_m * X_x_bar_ls[i][j];
      }
      w_m = kWm;
    }

    Allocator<float>::deallocate(X_x_ls_data, kXxLsLen * x_dim_);
  }

  /* 协方差权重 */
  const float kWc = 1 / (2 * (Na_ + lamda_));
  const float kWc0 = lamda_ / (lamda_ + Na_) + 1 - alpha_ * alpha_ +
                     beta_ + R_dim_ / (lamda_ + Na_);
  memset(P_bar, 0, sizeof(float) * x_dim_ * x_dim_);

  float w_c = kWc0;
  for (size_t i = 0; i < kXxBarLsLen; i++) {
    for (size_t j = 0; j < x_dim_; j++) {    // 行
      for (size_t k = 0; k < x_dim_; k++) {  // 列
        P_bar[j * x_dim_ + k] += w_c * (X_x_bar_ls[i][j] - x_hat_bar[j]) *
                                 (X_x_bar_ls[i][k] - x_hat_bar[k]);
      }
    }
    w_c = kWc;
  }
}

void Ukf::update(const float z[], const float x_hat_bar[], const float* P_bar,
                 const float* X_x_bar)
{
  /* 变量检查 */
#pragma reginon
  HW_ASSERT(z != nullptr, "z must not be nullptr");
  HW_ASSERT(x_hat_bar != nullptr, "x_hat_bar must not be nullptr");
  HW_ASSERT(P_bar != nullptr, "P_bar must not be nullptr");
  HW_ASSERT(X_x_bar != nullptr, "X_x_bar must not be nullptr");
#pragma endreginon

  float* Pzz = Allocator<float>::allocate(z_dim_ * z_dim_);
  float* K = Allocator<float>::allocate(x_dim_ * z_dim_);
  arm_matrix_instance_f32 K_mat, Pzz_mat;
  {
    float* Pxz = Allocator<float>::allocate(x_dim_ * z_dim_);
    float* z_hat = Allocator<float>::allocate(z_dim_);
    getOutputStatData(x_hat_bar, P_bar, X_x_bar, Pzz, Pxz, z_hat);

    float* Pzz_inv = Allocator<float>::allocate(z_dim_ * z_dim_);
    float* Pzz_tmp = Allocator<float>::allocate(z_dim_ * z_dim_);
    memcpy(Pzz_tmp, Pzz, sizeof(float) * z_dim_ * z_dim_);
    arm_matrix_instance_f32 Pxz_mat, Pzz_inv_mat;
    arm_mat_init_f32(&K_mat, x_dim_, z_dim_, K);
    arm_mat_init_f32(&Pzz_mat, z_dim_, z_dim_, Pzz);
    arm_mat_init_f32(&Pxz_mat, x_dim_, z_dim_, Pxz);
    arm_mat_init_f32(&Pzz_inv_mat, z_dim_, z_dim_, Pzz_inv);
    arm_mat_inverse_f32(&Pzz_mat, &Pzz_inv_mat);
    memcpy(Pzz, Pzz_tmp, sizeof(float) * z_dim_ * z_dim_);
    arm_mat_mult_f32(&Pxz_mat, &Pzz_inv_mat, &K_mat);

    float* err = Allocator<float>::allocate(z_dim_);
    float* delta_x = Allocator<float>::allocate(x_dim_);
    arm_sub_f32(z, z_hat, err, z_dim_);
    arm_mat_vec_mult_f32(&K_mat, err, delta_x);
    arm_add_f32(x_hat_bar, delta_x, x_hat_, x_dim_);

    Allocator<float>::deallocate(delta_x, x_dim_);
    Allocator<float>::deallocate(err, z_dim_);
    Allocator<float>::deallocate(Pzz_tmp, z_dim_ * z_dim_);
    Allocator<float>::deallocate(Pzz_inv, z_dim_ * z_dim_);
    Allocator<float>::deallocate(z_hat, z_dim_);
    Allocator<float>::deallocate(Pxz, x_dim_ * z_dim_);
  }

  float* K_T = Allocator<float>::allocate(z_dim_ * x_dim_);
  float* tmp1 = Allocator<float>::allocate(x_dim_ * z_dim_);
  float* tmp2 = Allocator<float>::allocate(x_dim_ * x_dim_);
  arm_matrix_instance_f32 K_T_mat, tmp1_mat, tmp2_mat;
  arm_mat_init_f32(&K_T_mat, z_dim_, x_dim_, K_T);
  arm_mat_init_f32(&tmp1_mat, x_dim_, z_dim_, tmp1);
  arm_mat_init_f32(&tmp2_mat, x_dim_, x_dim_, tmp2);
  arm_mat_trans_f32(&K_mat, &K_T_mat);
  arm_mat_mult_f32(&K_mat, &Pzz_mat, &tmp1_mat);
  arm_mat_mult_f32(&tmp1_mat, &K_T_mat, &tmp2_mat);
  arm_sub_f32(P_bar, tmp2, P_, x_dim_ * x_dim_);

  Allocator<float>::deallocate(tmp2, x_dim_ * x_dim_);
  Allocator<float>::deallocate(tmp1, x_dim_ * z_dim_);
  Allocator<float>::deallocate(K_T, z_dim_ * x_dim_);

  Allocator<float>::deallocate(K, x_dim_ * z_dim_);
  Allocator<float>::deallocate(Pzz, z_dim_ * z_dim_);
}

void Ukf::getOutputStatData(
    const float x_hat_bar[], const float* P_bar, const float* X_x_bar,
    float* Pzz, float* Pxz, float z_hat[])
{
  /* 变量检查 */
#pragma reginon
  HW_ASSERT(x_hat_bar != nullptr, "x_hat_bar must not be nullptr");
  HW_ASSERT(P_bar != nullptr, "P_bar must not be nullptr");
  HW_ASSERT(X_x_bar != nullptr, "X_x_bar must not be nullptr");
  HW_ASSERT(Pzz != nullptr, "Pzz must not be nullptr");
  HW_ASSERT(Pxz != nullptr, "Pxz must not be nullptr");
  HW_ASSERT(z_hat != nullptr, "z_hat must not be nullptr");
#pragma endreginon
  const size_t kXxBarLsLen = 2 * (x_dim_ + Q_dim_) + 1;
  const size_t kZHatLsLen = 2 * Na_ + 1;

  const float(*X_x_bar_ls)[x_dim_] =
      reinterpret_cast<const float(*)[x_dim_]>(X_x_bar);

  /* 获取Pzz，Pxz与z_hat */
  /* 含噪声影响的下一时刻sigma点的观察值 */
  float* Z_hat_ls_data = Allocator<float>::allocate(kZHatLsLen * z_dim_);
  float(*Z_hat_ls)[z_dim_] = reinterpret_cast<float(*)[z_dim_]>(Z_hat_ls_data);

  /* 平均值权重 */
  const float kWm = 1 / (2 * (Na_ + lamda_));
  const float kWm0 = lamda_ / (lamda_ + Na_);
  memset(z_hat, 0, sizeof(float) * z_dim_);

  float w_m = kWm0;
  for (size_t i = 0; i < kZHatLsLen; i++) {
    if (i < kXxBarLsLen) {
      h_(X_x_bar_ls[i], v_mean_, Z_hat_ls[i]);
    } else {
      /* 噪声部分 */
      h_(X_x_bar_ls[0], X_v_ + (i - kXxBarLsLen) * R_dim_,
         Z_hat_ls[i]);
    }

    for (size_t j = 0; j < z_dim_; j++) {
      z_hat[j] += w_m * Z_hat_ls[i][j];
    }
    w_m = kWm;
  }

  /* 协方差权重 */
  const float kWc = 1 / (2 * (Na_ + lamda_));
  const float kWc0 = lamda_ / (lamda_ + Na_) + 1 - alpha_ * alpha_ +
                     beta_;
  memset(Pzz, 0, sizeof(float) * z_dim_ * z_dim_);
  memset(Pxz, 0, sizeof(float) * x_dim_ * z_dim_);

  float w_c = kWc0;
  for (size_t i = 0; i < kZHatLsLen; i++) {
    for (size_t k = 0; k < z_dim_; k++) {    // 列
      for (size_t j = 0; j <= k; j++) {      // 行
        Pzz[j * z_dim_ + k] +=
            w_c * (Z_hat_ls_data[i * z_dim_ + j] - z_hat[j]) *
            (Z_hat_ls_data[i * z_dim_ + k] - z_hat[k]);
        if (j != k) {
          Pzz[k * z_dim_ + j] = Pzz[j * z_dim_ + k];
        }
      }

      for (size_t j = 0; j < x_dim_; j++) {  // 行
        if (i < kXxBarLsLen) {
          Pxz[j * z_dim_ + k] += w_c * (X_x_bar_ls[i][j] - x_hat_bar[j]) *
                                 (Z_hat_ls_data[i * z_dim_ + k] - z_hat[k]);
        } else {
          /* 噪声部分 */
          Pxz[j * z_dim_ + k] += w_c * (X_x_bar_ls[0][j] - x_hat_bar[j]) *
                                 (Z_hat_ls_data[i * z_dim_ + k] - z_hat[k]);
        }
      }
    }
    w_c = kWc;
  }

  Allocator<float>::deallocate(Z_hat_ls_data, kZHatLsLen * z_dim_);
}
/* Private function definitions ----------------------------------------------*/

static void CholeskyDecomposition(const size_t n, float* A)
{
  /* 变量检查 */
#pragma reginon
  HW_ASSERT(A != nullptr, "A must not be nullptr");
  HW_ASSERT(n > 0, "n must be greater than 0");
#pragma endreginon

  float(*A_mat)[n] = reinterpret_cast<float(*)[n]>(A);
  for (size_t i = 0; i < n; i++) {    // 行
    for (size_t j = 0; j <= i; j++) {  // 列
      float sum = 0;
      if (i > j) {  // 下三角部分
        for (size_t k = 0; k < j; k++) {
          sum += A_mat[i][k] * A_mat[j][k];
        }
        A_mat[i][j] = (A_mat[i][j] - sum) / A_mat[j][j];
      } else {  // 对角线
        for (size_t k = 0; k < i; k++) {
          sum += A_mat[i][k] * A_mat[i][k];
        }
        A_mat[i][i] = sqrtf(A_mat[i][i] - sum);
      }
    }
    /* 上三角部分 */
    memset(A_mat[i] + i + 1, 0, sizeof(float) * (n - i - 1));
  }
}

HW_OPTIMIZE_O2_END
}  // namespace observer
}  // namespace hello_world