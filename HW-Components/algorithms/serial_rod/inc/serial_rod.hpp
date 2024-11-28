/**
 *******************************************************************************
 * @file      : serial_rod.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_SERIAL_ROD_SERIAL_ROD_HPP_
#define HW_COMPONENTS_ALGORITHMS_SERIAL_ROD_SERIAL_ROD_HPP_

/* Includes ------------------------------------------------------------------*/
#include "Eigen/Dense"
#include "allocator.hpp"
#include "system.hpp"

namespace hello_world
{
namespace serial_rod
{
/* Exported macro ------------------------------------------------------------*/

enum class JointType {
  kRevolute,   ///< 旋转关节
  kPrismatic,  ///< 平移关节
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
HW_OPTIMIZE_O2_START

struct JointParams : public MemMgr {
  float a = 0;      ///< 连杆长度，单位：m
  float alpha = 0;  ///< 连杆偏转角，单位：rad
  /** 连杆偏移量，当关节类型为平移关节时，实际的连杆偏移量为 d + q，单位：m */
  float d = 0;
  /** 连杆旋转角，当关节类型为旋转关节时，实际的连杆旋转角为 theta + q，单位：rad */
  float theta = 0;
  JointType type = JointType::kRevolute;  ///< 关节类型
};

class SerialRod : public MemMgr
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  SerialRod(void) = default;
  /**
   * @brief       串联连杆机构初始化
   * @param        joint_params_arr: 串联连杆机构的关节参数列表，由低到高，长度为
   *               joint_num
   * @param        joint_num: 串联连杆机构的关节数量
   * @retval       None
   * @note        None
   */
  SerialRod(JointParams joint_params_arr[], size_t joint_num);
  SerialRod(const SerialRod& other);
  SerialRod& operator=(const SerialRod& other);
  SerialRod(SerialRod&& other);
  SerialRod& operator=(SerialRod&& other);

  ~SerialRod(void);

  /* 配置方法 */
  /**
   * @brief       串联连杆机构初始化
   * @param        joint_params_arr: 串联连杆机构的关节参数列表，由低到高，长度为
   *               joint_num
   * @param        joint_num: 串联连杆机构的关节数量
   * @retval       None
   * @note        None
   */
  void init(JointParams joint_params_arr[], size_t joint_num);

  /* 功能性方法 */

  /**
   * @brief       正运动学求解
   * @param        q: 关节角度数组，长度为 joint_num
   * @param        tf: 末端执行器相对于基坐标系的变换矩阵
   * @retval       None
   * @note        None
   */
  void fwdKinematics(const float q[], Eigen::Isometry3f& tf) const;

  /**
   * @brief       正运动学求解
   * @param        q: 关节角度数组，长度为 joint_num
   * @param        tf_arr: 各关节坐标系相对于基坐标系的变换矩阵数组，由低到高，长度为
   *               joint_num
   * @retval       None
   * @note        None
   */
  void getTf2BaseArr(const float q[], Eigen::Isometry3f tf_arr[]) const;

  /**
   * @brief       雅克比矩阵求解
   * @param        q: 关节角度数组，长度为 joint_num
   * @param        jacobian: 雅克比矩阵，大小为 6 x joint_num
   * @retval       None
   * @note        None
   */
  void jacobian(const float q[], Eigen::MatrixXf& jacobian) const;

 private:
  /* 功能性方法 */

  /**
   * @brief       计算变换矩阵
   * @param        joint_params: 关节参数
   * @param        tf: 变换矩阵
   * @retval       None
   * @note        None
   */
  void calcIsometry(
      const JointParams& joint_params, Eigen::Isometry3f& tf) const;

  size_t joint_num_ = 0;  ///< 串联连杆机构的关节数量
  /** 串联连杆机构的关节参数列表 */
  JointParams* joint_params_arr_ = nullptr;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace serial_rod
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_SERIAL_ROD_SERIAL_ROD_HPP_ */
