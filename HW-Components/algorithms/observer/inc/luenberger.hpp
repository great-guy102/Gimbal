/**
 *******************************************************************************
 * @file      : luenberger.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_OBSERVER_LUENBERGER_HPP_
#define HW_COMPONENTS_ALGORITHMS_OBSERVER_LUENBERGER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "arm_math.h"
#include "observer_base.hpp"
#include "system.hpp"
#include "allocator.hpp"

namespace hello_world
{
namespace observer
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START

struct LuenbergerConfig : public MemMgr{
  size_t x_dim = 0;  //!< 状态量维度
  size_t z_dim = 0;  //!< 观测量维度
  size_t u_dim = 0;  //!< 控制量维度

  float* F = nullptr;   //!< 系统矩阵
  float* B = nullptr;   //!< 控制矩阵
  float* H = nullptr;   //!< 观测矩阵
  float* L = nullptr;   //!< 龙伯格增益矩阵
  float* x0 = nullptr;  //!< 初始状态，不使用时置为nullptr，此时设置为零状态
};

class Luenberger : public Observer
{
 public:
  typedef LuenbergerConfig Config;

  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  Luenberger(void) = default;
  /**
   * @brief       龙伯格观测器初始化（离散）
   * @param        config_d: 离散系统配置，初始化后可释放
   * @retval       None
   * @note        配置参数中的矩阵需要为离散系统的矩阵
   */
  Luenberger(const Config& config_d);
  /**
   * @brief       龙伯格观测器初始化（连续）
   * @param        config_c: 连续系统配置，初始化后可释放
   * @param        samp_period: 离散化时间间隔
   * @retval       None
   * @note        配置参数中的矩阵需要为连续系统的矩阵，使用前向欧拉法离散化
   */
  Luenberger(const Config& config_c, float samp_period);
  Luenberger(const Luenberger& other);
  Luenberger& operator=(const Luenberger& other);
  Luenberger(Luenberger&& other);
  Luenberger& operator=(Luenberger&& other);

  virtual ~Luenberger(void);

  /* 重载方法 */

  /**
   * @brief       观测计算
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
  virtual void reset(void) override
  {
    memset(x_hat_, 0, x_dim_ * sizeof(float));
  }

  /**
   * @brief       设置初始状态
   * @param        x0: 初始状态，大小为 x_dim_
   * @retval       None
   * @note        None
   */
  virtual void setX0(const float x0[]) override;

  /* 配置方法 */

  /**
   * @brief       龙伯格观测器初始化（离散），使用默认构造函数后请务必调用此函数
   * @param        config_d: 离散系统配置，初始化后可释放
   * @retval       None
   * @note        配置参数中的矩阵需要为离散系统的矩阵
   */
  void init(const Config& config_d);

  /**
   * @brief       龙伯格观测器初始化（连续），使用默认构造函数后请务必调用此函数
   * @param        config_c: 连续系统配置，初始化后可释放
   * @param        samp_period: 离散化时间间隔
   * @retval       None
   * @note        配置参数中的矩阵需要为连续系统的矩阵，使用前向欧拉法离散化
   */
  void init(const Config& config_c, float samp_period);

 private:
  float* Fd_ = nullptr;
  arm_matrix_instance_f32 Fd_mat_;  //!< 离散系统矩阵
  float* Bd_ = nullptr;
  arm_matrix_instance_f32 Bd_mat_;  //!< 离散控制矩阵
  float* Hd_ = nullptr;
  arm_matrix_instance_f32 Hd_mat_;  //!< 离散观测矩阵
  float* Ld_ = nullptr;
  arm_matrix_instance_f32 Ld_mat_;  //!< 离散龙伯格增益矩阵
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace observer
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_OBSERVER_LUENBERGER_HPP_ */
