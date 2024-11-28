/**
 *******************************************************************************
 * @file      : ukf.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_OBSERVER_UKF_HPP_
#define HW_COMPONENTS_ALGORITHMS_OBSERVER_UKF_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "arm_math.h"
#include "observer.hpp"
#include "system.hpp"

namespace hello_world
{
namespace observer
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START

class Ukf : public Observer
{
 public:
  /**
   * @brief       状态方程 x_k = f(x_(k-1), u_k, w_k)
   * @param        x_k_1: x_(k-1)，大小为 x_dim_
   * @param        u_k: u_k，大小为 u_dim_
   * @param        w_k: w_k，大小为 Q_dim_
   * @param        x_k: x_k，大小为 x_dim_
   * @retval       None
   * @note        None
   */
  typedef void (*pStateFunc)(
      const float x_k_1[], const float u_k[], const float w_k[], float x_k[]);

  /**
   * @brief       输出方程 z_k = h(x_k, v_k)
   * @param        x_k: x_k，大小为 x_dim_
   * @param        v_k: v_k，大小为 R_dim_
   * @param        z_k: z_k，大小为 z_dim_
   * @retval       None
   * @note        None
   */
  typedef void (*pOutputFunc)(
      const float x_k[], const float v_k[], float z_k[]);

  struct Config : public MemMgr {  //!< 噪声非线性观测器配置
    size_t x_dim = 0;  //!< 状态量维度
    size_t z_dim = 0;  //!< 观测量维度
    size_t u_dim = 0;  //!< 控制量维度

    size_t Q_dim = 0;  //!< 状态噪声协方差矩阵维度
    size_t R_dim = 0;  //!< 观测噪声协方差矩阵维度

    float* Q = nullptr;   //!< 状态噪声协方差矩阵
    float* R = nullptr;   //!< 观测噪声协方差矩阵
    float* P = nullptr;   //!< 状态估计协方差矩阵，不可为零矩阵
    float* x0 = nullptr;  //!< 初始状态，不使用时置为 nullptr ，此时设置为零状态

    float alpha = 0.001f;  //!< 用于确定 sigma 点的分布，太小容易导致发散，[0, 1]
    float beta = 2.0f;     //!< 用于确定高斯分布的权重，>= 0
    float kappa = 0.0f;    //!< 用于确定高斯分布的权重，>= 0

    pStateFunc f = nullptr;   //!< 状态方程
    pOutputFunc h = nullptr;  //!< 输出方程
  };

  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  Ukf(void) = default;
  /**
   * @brief       无迹卡尔曼滤波器初始化
   * @param        config: 离散系统配置，初始化后可释放
   * @retval       None
   * @note        配置参数中的矩阵需要为离散系统的矩阵
   */
  Ukf(const Config& config);
  Ukf(const Ukf& other);
  Ukf& operator=(const Ukf& other);
  Ukf(Ukf&& other);
  Ukf& operator=(Ukf&& other);

  virtual ~Ukf(void);

  /* 重载方法 */

  /**
   * @brief       无迹卡尔曼滤波器计算
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
   * @param        x0: 初始状态，大小为 x_dim
   * @retval       None
   * @note        None
   */
  void setX0(const float x0[]) override;

  /* 配置方法 */

  /**
   * @brief       无迹卡尔曼滤波器初始化，使用默认构造函数后请务必调用此函数
   * @param        config: 离散系统配置，初始化后可释放
   * @retval       None
   * @note        配置参数中的矩阵需要为离散系统的矩阵
   */
  void init(const Config& config);

 private:
  /**
   * @brief       获取 sigma 点
   * @param        x: 状态量，大小为 n
   * @param        P: 状态估计协方差矩阵，大小为 (n * n)，按行存储
   * @param        n: 状态量维度
   * @param        scale: 缩放因子
   * @param        X: 返回 sigma 点，大小为 (2 * n + 1, n)，按行存储
   * @retval       None
   * @note        None
   */
  void getSigmaPnts(const float x[], const float* P, size_t n, float scale,
                    float* X) const;

  /**
   * @brief       预测
   * @param        u: 控制量，大小为 u_dim_
   * @param        x_hat_bar: 返回预测后的状态量，大小为 x_dim_
   * @param        P_bar: 返回预测后的状态估计协方差矩阵，大小为 (x_dim_, x_dim_)，
   *               按行存储
   * @param        X_x_bar: 返回预测后的状态量的 sigma 点，大小为
   *               (2 * (x_dim_ + Q_dim_) + 1, x_dim_)，按行存储
   * @retval       None
   * @note        None
   */
  void predict(const float u[], float x_hat_bar[], float* P_bar,
               float* X_x_bar) const;

  /**
   * @brief       更新
   * @param        z: 观测量，大小为 z_dim_
   * @param        x_hat_bar: 预测后的状态量，大小为 x_dim_
   * @param        P_bar: 预测后的状态估计协方差矩阵，大小为 (x_dim_, x_dim_)，按行
   *               存储
   * @param        X_x_bar: 预测后的状态量的 sigma 点，大小为
   *               (2 * (x_dim_ + Q_dim_) + 1, x_dim_)，按行存储
   * @retval       None
   * @note        None
   */
  void update(const float z[], const float x_hat_bar[], const float* P_bar,
              const float* X_x_bar);

  /**
   * @brief       获取估计输出统计数据
   * @param        x_hat_bar: 预测后的状态量，大小为 x_dim_
   * @param        P_bar: 预测后的状态估计协方差矩阵，大小为 (x_dim_, x_dim_)，按行
   *               存储
   * @param        X_x_bar: 预测后的状态量的 sigma 点，大小为
   *               (2 * (x_dim_ + Q_dim_) + 1, x_dim_)，按行存储
   * @param        Pzz: 返回 Pzz，大小为 (z_dim_, z_dim_)，按行存储
   * @param        Pxz: 返回 Pxz，大小为 (x_dim_, z_dim_)，按行存储
   * @param        z_hat: 返回 z_hat，大小为 z_dim_
   * @retval       None
   * @note        None
   */
  void getOutputStatData(const float x_hat_bar[], const float* P_bar,
                         const float* X_x_bar, float* Pzz, float* Pxz,
                         float z_hat[]);

  float alpha_ = 0.001f;
  float beta_ = 2.0f;
  float kappa_ = 0.0f;

  float* Q_ = nullptr;
  float* R_ = nullptr;
  float* P_ = nullptr;

  float* X_w_ = nullptr;     //!< 状态噪声sigma点
  float* X_v_ = nullptr;     //!< 观测噪声sigma点
  float* w_mean_ = nullptr;  //!< 状态噪声均值
  float* v_mean_ = nullptr;  //!< 观测噪声均值

  pStateFunc f_ = nullptr;
  pOutputFunc h_ = nullptr;

  size_t Q_dim_ = 0;
  size_t R_dim_ = 0;
  size_t Na_ = 0;  //!< 增广状态量维度

  float lamda_ = 0;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace observer
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_OBSERVER_UKF_HPP_ */
