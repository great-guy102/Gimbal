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
#include "module_fsm.hpp"
#include "module_state.hpp"

/* Exported macro ------------------------------------------------------------*/

namespace robot {
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

union ChassisState {
  struct {
    float v_x;
    float v_y;
    float w;
  };
  float data[3];
  void reset() {
    v_x = 0;
    v_y = 0;
    w = 0;
  }
  ChassisState operator+(const ChassisState &other) const {
    return {v_x + other.v_x, v_y + other.v_y, w + other.w};
  }

  ChassisState operator-(const ChassisState &other) const {
    return {v_x - other.v_x, v_y - other.v_y, w - other.w};
  }

  ChassisState operator*(float scalar) const {
    return {v_x * scalar, v_y * scalar, w * scalar};
  }

  ChassisState operator+=(const ChassisState &other) {
    v_x += other.v_x;
    v_y += other.v_y;
    w += other.w;
    return *this;
  }

  ChassisState operator-=(const ChassisState &other) {
    v_x -= other.v_x;
    v_y -= other.v_y;
    w -= other.w;
    return *this;
  }

  ChassisState operator*=(float scalar) {
    v_x *= scalar;
    v_y *= scalar;
    w *= scalar;
    return *this;
  }

  friend ChassisState operator*(float scalar, const ChassisState &state);
};

class Chassis : public hello_world::MemMgr {
public:
  typedef ChassisWorkingMode WorkingMode;
  typedef hello_world::module::CtrlMode CtrlMode;
  typedef ChassisState ChassisCmd;

  static std::string WorkingModeToStr(WorkingMode mode) {
    if (mode == Chassis::WorkingMode::Depart)
      return "Depart";
    if (mode == Chassis::WorkingMode::Follow)
      return "Follow";
    if (mode == Chassis::WorkingMode::Gyro)
      return "Gyro";
    return "Unknown";
  };

  enum class GyroDir : int8_t {
    Clockwise = -1,    ///< 顺时针
    Unspecified = 0,   ///< 静止
    AntiClockwise = 1, ///< 逆时针
  };

  enum class GyroMode : int8_t {
    ConstW = 0, ///< 常速旋转
    SinW = 1,   ///< 正弦偏置速度旋转
  };

  Chassis() {};
  ~Chassis() {};

  void setWorkingMode(WorkingMode mode);
  WorkingMode getWorkingMode() const { return working_mode_; }

  void setCtrlMode(CtrlMode mode) { ctrl_mode_ = mode; }
  CtrlMode getCtrlMode() const { return ctrl_mode_; }

  void setNormCmd(const ChassisCmd &cmd) { cmd_norm_ = cmd; }
  ChassisCmd getNormCmd() const { return cmd_norm_; }

  void setGyroDir(GyroDir dir);
  GyroDir getGyroDir() const { return gyro_dir_; }

  void setGyroMode(GyroMode mode) { gyro_mode_ = mode; }
  GyroMode getGyroMode() const { return gyro_mode_; }

  void setUseCapFlag(bool flag) { use_cap_flag_ = flag; }
  bool getUseCapFlag() const { return use_cap_flag_; }

  void setRevChassisCnt(uint8_t cnt) { rev_chassis_cnt_ = cnt; }
  uint8_t getRevChassisCnt() const { return rev_chassis_cnt_; }

  void setRevGimbalCnt(uint8_t cnt) { rev_chassis_cnt_ = cnt; }
  uint8_t getRevGimbalCnt() const { return rev_chassis_cnt_; }

private:
  // 由 robot 设置的数据
  WorkingMode working_mode_ = WorkingMode::Depart; ///< 工作模式
  CtrlMode ctrl_mode_ = CtrlMode::kManual;         ///< 控制模式
  ChassisCmd cmd_norm_ = {0};               ///< 原始控制指令，基于图传坐标系
  GyroDir gyro_dir_ = GyroDir::Unspecified; ///< 小陀螺方向
  GyroMode gyro_mode_ = GyroMode::ConstW;   ///< 陀螺模式
  ///< 小陀螺方向，正为绕 Z 轴逆时针，负为顺时针，

  bool use_cap_flag_ = false; ///< 是否使用超级电容
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
