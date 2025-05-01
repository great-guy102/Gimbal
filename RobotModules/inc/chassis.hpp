/**
 *******************************************************************************
 * @file      :chassis.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOT_MODULES_CHASSIS_HPP_
#define ROBOT_MODULES_CHASSIS_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "module_state.hpp"

/* Exported macro ------------------------------------------------------------*/

namespace robot {
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Chassis : public hello_world::MemMgr {
public:
  typedef robot::ChassisState ChassisCmd;
  typedef robot::ChassisWorkingMode WorkingMode;
  typedef robot::GyroDir GyroDir;
  typedef robot::GyroMode GyroMode;

  Chassis() {};
  ~Chassis() {};

  void setWorkingMode(WorkingMode mode){ working_mode_ = mode; };
  WorkingMode getWorkingMode() const { return working_mode_; }

  void setNormCmd(const ChassisCmd &cmd) { cmd_norm_ = cmd; }
  ChassisCmd getNormCmd() const { return cmd_norm_; }

  void setGyroDir(GyroDir dir){ gyro_dir_ = dir; };
  GyroDir getGyroDir() const { return gyro_dir_; }

  void setGyroMode(GyroMode mode) { gyro_mode_ = mode; }
  GyroMode getGyroMode() const { return gyro_mode_; }

  void setUseCapFlag(bool flag) { use_cap_flag_ = flag; }
  bool getUseCapFlag() const { return use_cap_flag_; }

  void setRevChassisCnt(uint8_t cnt) { rev_chassis_cnt_ = cnt; }
  uint8_t getRevChassisCnt() const { return rev_chassis_cnt_; }

  void setRevGimbalCnt(uint8_t cnt) { rev_gimbal_cnt_ = cnt; }
  uint8_t getRevGimbalCnt() const { return rev_gimbal_cnt_; }

private:
  // 由 robot 设置的数据
  WorkingMode working_mode_ = WorkingMode::Depart; ///< 工作模式
  ChassisCmd cmd_norm_ = {0};               ///< 原始控制指令，基于图传坐标系
  GyroDir gyro_dir_ = GyroDir::Unspecified; ///< 小陀螺方向
  GyroMode gyro_mode_ = GyroMode::ConstW;   ///< 陀螺模式
  ///< 小陀螺方向，正为绕 Z 轴逆时针，负为顺时针，

  bool use_cap_flag_ = false;   ///< 是否使用超级电容
  uint8_t rev_chassis_cnt_ = 0; ///< 翻转底盘朝向标志位
  uint8_t rev_gimbal_cnt_ = 0;  ///< 翻转云台朝向标志位
};

/* Exported variables
 * --------------------------------------------------------*/

/* Exported function prototypes
 * ----------------------------------------------*/

inline ChassisState operator*(float scalar, const ChassisState &cmd) {
  return cmd * scalar;
};

} // namespace robot
#endif /* ROBOT_MODULES_CHASSIS_HPP_ */
