/**
 *******************************************************************************
 * @file      : observer_base.hpp
 * @brief     : 观测器基类
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
#ifndef HW_COMPONENTS_ALGORITHMS_OBSERVER_OBSERVER_BASE_HPP_
#define HW_COMPONENTS_ALGORITHMS_OBSERVER_OBSERVER_BASE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstddef>
#include <cstring>

#include "allocator.hpp"
#include "system.hpp"

namespace hello_world
{
namespace observer
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START

class Observer : public MemMgr
{
 public:
  Observer(void) = default;
  /**
   * @brief       观测器初始化
   * @param        x_dim: 状态量维度
   * @param        z_dim: 观测量维度
   * @param        u_dim: 控制量维度
   * @retval       None
   * @note        None
   */
  Observer(size_t x_dim, size_t z_dim, size_t u_dim);
  Observer(const Observer& other);
  Observer& operator=(const Observer& other);
  Observer(Observer&& other);
  Observer& operator=(Observer&& other);

  virtual ~Observer(void);

  /* 功能性方法 */

  /**
   * @brief       观测计算
   * @param        u: 控制量，大小为 u_dim_
   * @param        z: 观测量，大小为 z_dim_
   * @retval       None
   * @note        None
   */
  virtual void calc(const float u[], const float z[]) = 0;

  /**
   * @brief       重置
   * @param        None
   * @retval       None
   * @note        None
   */
  virtual void reset(void) = 0;

  /* 数据修改与获取 */

  /**
   * @brief       获取最优状态估计量
   * @param        x_hat: 最优状态估计量，大小为 x_dim_
   * @retval       None
   * @note        None
   */
  void getX(float x_hat[]) const;

  /**
   * @brief       设置初始状态
   * @param        x0: 初始状态，大小为 x_dim
   * @retval       None
   * @note        None
   */
  virtual void setX0(const float x0[]) = 0;

  size_t x_dim(void) const { return x_dim_; }

  size_t z_dim(void) const { return z_dim_; }

  size_t u_dim(void) const { return u_dim_; }

 protected:
  /* 配置方法 */

  /**
   * @brief       观测器初始化，使用默认构造函数后请务必调用此函数
   * @param        x_dim: 状态量维度
   * @param        z_dim: 观测量维度
   * @param        u_dim: 控制量维度
   * @retval       None
   * @note        None
   */
  void init(size_t x_dim, size_t z_dim, size_t u_dim);

  size_t x_dim_ = 0;  ///< 状态量维数
  size_t z_dim_ = 0;  ///< 观测量维数
  size_t u_dim_ = 0;  ///< 控制量维数

  float* x_hat_ = nullptr;  ///< 最优状态估计量
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace observer
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_OBSERVER_OBSERVER_BASE_HPP_ */
