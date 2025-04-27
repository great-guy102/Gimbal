/**
 *******************************************************************************
 * @file      :shooter.hpp
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
#ifndef ROBOT_MODULES_SHOOTER_HPP_
#define ROBOT_MODULES_SHOOTER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "module_fsm.hpp"
#include "module_state.hpp"

/* Exported macro ------------------------------------------------------------*/
namespace robot {
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class Shooter : public hello_world::MemMgr {
public:
  typedef ShooterWorkingMode WorkingMode;
  typedef hello_world::module::CtrlMode CtrlMode;

  static std::string WorkingModeToStr(WorkingMode mode) {
    if (mode == WorkingMode::kStop)
      return "Stop";
    else if (mode == WorkingMode::kShoot)
      return "Shoot";
    else if (mode == WorkingMode::kBackward)
      return "Backward";
    return "Unknown";
  };

  Shooter() {};
  ~Shooter() {};

  void setCtrlMode(CtrlMode mode) { ctrl_mode_ = mode; }
  CtrlMode getCtrlMode() const { return ctrl_mode_; }

  void setWorkingMode(WorkingMode mode) { working_mode_ = mode; }
  WorkingMode getWorkingMode() const { return working_mode_; }

  void setShootFlag(bool flag) {
    if (flag) {
      shoot_cnt_++;
    }
  }
  void shoot() { setShootFlag(true); };
  void clearShootFlag() { last_shoot_cnt_ = shoot_cnt_; }
  bool getShootFlag(bool auto_clear = true) {
    bool res = shoot_cnt_ != last_shoot_cnt_;
    if (auto_clear) {
      clearShootFlag();
    }
    return res;
  }

private:
  // 由 robot 设置的数据
  uint8_t shoot_cnt_ = 0, last_shoot_cnt_ = 0;

  CtrlMode ctrl_mode_ = CtrlMode::kManual;
  WorkingMode working_mode_ = WorkingMode::kShoot;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
} // namespace robot
#endif /* ROBOT_MODULE_SHOOTER_HPP_ */
