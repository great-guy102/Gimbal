/**
 *******************************************************************************
 * @file      : kalman.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_OBSERVER_KALMAN_HPP_
#define HW_COMPONENTS_ALGORITHMS_OBSERVER_KALMAN_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "arm_math.h"
#include "observer_base.hpp"
#include "system.hpp"

namespace hello_world
{
namespace observer
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START

struct KalmanConfig : public MemMgr {
  size_t x_dim = 0;  //!< 状态量维度
  size_t z_dim = 0;  //!< 观测量维度
  size_t u_dim = 0;  //!< 控制量维度

  float* F = nullptr;   //!< 系统矩阵
  float* B = nullptr;   //!< 控制矩阵
  float* H = nullptr;   //!< 观测矩阵
  float* Q = nullptr;   //!< 状态噪声协方差矩阵
  float* R = nullptr;   //!< 观测噪声协方差矩阵
  /*<! 状态估计协方差矩阵，不使用时置为 nullptr，此时设置为零状态 */
  float* P = nullptr;
  float* x0 = nullptr;  //!< 初始状态，不使用时置为 nullptr，此时设置为零状态
};

class Kalman : public Observer
{
 public:
  typedef KalmanConfig Config;

  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  Kalman(void) = default;
  /**
   * @brief       卡尔曼滤波器初始化
   * @param        config: 离散系统配置，初始化后可释放
   * @retval       None
   * @note        配置参数中的矩阵需要为离散系统的矩阵
   */
  Kalman(const Config& config);
  Kalman(const Kalman& other);
  Kalman& operator=(const Kalman& other);
  Kalman(Kalman&& other);
  Kalman& operator=(Kalman&& other);

  virtual ~Kalman(void);

  /* 重载方法 */

  /**
   * @brief       卡尔曼滤波器计算
   * @param        u: 控制量，大小为 u_dim_
   * @param        z: 观测量，大小为 z_dim_
   * @retval       None
   * @note        None
   */
  virtual void calc(const float u[], const float z[]) override;

  /**
   * @brief       重置
   * @param        None
   * @retval       None
   * @note        None
   */
  void reset(void) override
  {
    memset(x_hat_, 0, x_dim_ * sizeof(float));
    memset(P_, 0, x_dim_ * x_dim_ * sizeof(float));
  }

  /**
   * @brief       设置初始状态
   * @param        x0: 初始状态，大小为 x_dim_
   * @retval       None
   * @note        None
   */
  void setX0(const float x0[]) override;

  /* 配置方法 */

  /**
   * @brief       卡尔曼滤波器初始化，使用默认构造函数后请务必调用此函数
   * @param        config: 离散系统配置，初始化后可释放
   * @retval       None
   * @note        配置参数中的矩阵需要为离散系统的矩阵
   */
  void init(const Config& config);

 private:
  /* 功能性方法 */

  /**
   * @brief       卡尔曼滤波器预测
   * @param        u: 控制量，大小为 u_dim_
   * @param        x_hat_bar: 返回预测状态量，大小为 x_dim_
   * @param        P_bar_mat: 返回预测状态协方差矩阵，大小为 x_dim_ * x_dim_
   * @retval       None
   * @note        None
   */
  void predict(const float u[], float x_hat_bar[],
               arm_matrix_instance_f32& P_bar_mat) const;

  /**
   * @brief       卡尔曼滤波器校正
   * @param        z: 观测量，大小为 z_dim_
   * @param        x_hat_bar: 预测状态量，大小为 x_dim_
   * @param        P_bar_mat: 预测状态协方差矩阵，大小为 x_dim_ * x_dim_
   * @retval       None
   * @note        None
   */
  void update(const float z[], const float x_hat_bar[],
              const arm_matrix_instance_f32& P_bar_mat);

  float* F_ = nullptr;
  float* B_ = nullptr;
  float* H_ = nullptr;
  float* Q_ = nullptr;
  float* R_ = nullptr;
  float* P_ = nullptr;

  arm_matrix_instance_f32 F_mat_;
  arm_matrix_instance_f32 B_mat_;
  arm_matrix_instance_f32 H_mat_;
  arm_matrix_instance_f32 Q_mat_;
  arm_matrix_instance_f32 R_mat_;
  arm_matrix_instance_f32 P_mat_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace observer
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_OBSERVER_KALMAN_HPP_ */
