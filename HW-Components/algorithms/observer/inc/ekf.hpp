/**
 *******************************************************************************
 * @file      : ekf.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_OBSERVER_EKF_HPP_
#define HW_COMPONENTS_ALGORITHMS_OBSERVER_EKF_HPP_

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

class Ekf : public Observer
{
 public:
  /**
   * @brief       状态方程 x_k = f(x_(k-1), u_k)
   * @param        x_k_1: x_(k-1)，大小为 x_dim_
   * @param        u_k: u_k，大小为 u_dim_
   * @param        x_k: x_k，大小为 x_dim_
   * @retval       None
   * @note        None
   */
  typedef void (*pStateFunc)(
      const float x_k_1[], const float u_k[], float x_k[]);

  /**
   * @brief       输出方程 z_k = h(x_k)
   * @param        x_k: x_k，大小为 x_dim_
   * @param        z_k: z_k，大小为 z_dim_
   * @retval       None
   * @note        None
   */
  typedef void (*pOutputFunc)(const float x_k[], float z_k[]);

  /**
   * @brief       获取状态方程雅可比矩阵 J_F = partial f / partial x_(k-1) | (x_hat_k_1, u_k)
   * @param        x_hat_k_1: x_hat_(k-1)，大小为 x_dim_
   * @param        u_k: u_k，大小为 u_dim_
   * @param        J_F: J_F，大小为 (x_dim_, x_dim_)，按行存储
   * @retval       None
   * @note        None
   */
  typedef void (*pGetJ_F)(
      const float x_hat_k_1[], const float u_k[], float* J_F);

  /**
   * @brief       获取输出方程雅可比矩阵 J_H = partial h / partial x_k | (x_hat_k)
   * @param        x_hat_k: x_hat_k，大小为 x_dim_
   * @param        J_H: J_H，大小为 (z_dim_, x_dim_)，按行存储
   * @retval       None
   * @note        None
   */
  typedef void (*pGetJ_H)(const float x_hat_k[], float* J_H);

  struct Config : public MemMgr {
    size_t x_dim = 0;  //!< 状态量维度
    size_t z_dim = 0;  //!< 观测量维度
    size_t u_dim = 0;  //!< 控制量维度

    pStateFunc f = nullptr;    //!< 状态方程
    pOutputFunc h = nullptr;   //!< 输出方程
    pGetJ_F GetJ_F = nullptr;  //!< 获取状态方程雅可比矩阵
    pGetJ_H GetJ_H = nullptr;  //!< 获取输出方程雅可比矩阵

    float* Q = nullptr;   //!< 状态噪声协方差矩阵
    float* R = nullptr;   //!< 观测噪声协方差矩阵
    /*!< 状态估计协方差矩阵，不使用时置为 nullptr，此时设置为零状态 */
    float* P = nullptr;
    float* x0 = nullptr;  //!< 初始状态，不使用时置为 nullptr，此时设置为零状态
  };

  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  Ekf(void) = default;
  /**
   * @brief       扩展卡尔曼滤波器初始化
   * @param        config: 离散系统配置，初始化后可释放
   * @retval       None
   * @note        配置参数中的矩阵需要为离散系统的矩阵
   */
  Ekf(const Config& config);
  Ekf(const Ekf& other);
  Ekf& operator=(const Ekf& other);
  Ekf(Ekf&& other);
  Ekf& operator=(Ekf&& other);

  virtual ~Ekf(void);

  /* 重载方法 */

  /**
   * @brief       扩展卡尔曼滤波器计算
   * @param        u: 控制量，大小为 u_dim_
   * @param        z: 观测量，大小为 z_dim
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
   * @brief       扩展卡尔曼滤波器初始化，使用默认构造函数后请务必调用此函数
   * @param        config: 离散系统配置，初始化后可释放
   * @retval       None
   * @note        配置参数中的矩阵需要为离散系统的矩阵
   */
  void init(const Config& config);

 private:
  /* 功能性方法 */

  /**
   * @brief       扩展卡尔曼滤波器预测
   * @param        u: 控制量，大小为 u_dim_
   * @param        x_hat_bar: 返回预测状态量，大小为 x_dim_
   * @param        P_bar_mat: 返回预测状态协方差矩阵，大小为 (x_dim_, x_dim_)
   * @retval       None
   * @note        None
   */
  void predict(const float u[], float x_hat_bar[],
               arm_matrix_instance_f32& P_bar_mat) const;

  /**
   * @brief       扩展卡尔曼滤波器校正
   * @param        z: 观测量，大小为 z_dim_
   * @param        x_hat_bar: 预测状态量，大小为 x_dim_
   * @param        P_bar_mat: 预测状态协方差矩阵，大小为 (x_dim_, x_dim_)
   * @retval       None
   * @note        None
   */
  void update(const float z[], const float x_hat_bar[],
              const arm_matrix_instance_f32& P_bar_mat);

  pStateFunc f_ = nullptr;
  pOutputFunc h_ = nullptr;
  pGetJ_F GetJ_F_ = nullptr;
  pGetJ_H GetJ_H_ = nullptr;

  float* P_ = nullptr;
  float* Q_ = nullptr;
  float* R_ = nullptr;

  arm_matrix_instance_f32 P_mat_;
  arm_matrix_instance_f32 Q_mat_;
  arm_matrix_instance_f32 R_mat_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace observer
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_OBSERVER_EKF_HPP_ */
