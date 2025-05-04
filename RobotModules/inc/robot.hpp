/**
 *******************************************************************************
 * @file      :robot.hpp
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
#ifndef ROBOT_MODULE_ROBOT_HPP_
#define ROBOT_MODULE_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include "DT7.hpp"
#include "buzzer.hpp"
#include "fsm.hpp"
#include "imu.hpp"
#include "laser.hpp"
#include "motor.hpp"
#include "tick.hpp"
#include "vision.hpp"

#include "feed.hpp"
#include "fric_2motor.hpp"

#include "gimbal_chassis_comm.hpp"
#include "module_state.hpp"

#include "chassis.hpp"
#include "gimbal.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace robot {
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class Robot : public hello_world::module::ModuleFsm {
public:
  typedef hello_world::buzzer::Buzzer Buzzer;
  typedef hello_world::motor::Motor Motor;
  typedef hello_world::laser::Laser Laser;
  typedef hello_world::imu::Imu Imu;
  typedef hello_world::vision::Vision Vision;
  typedef hello_world::vision::Vision::ShootFlag VisionShootFlag;
  typedef hello_world::remote_control::DT7 DT7;
  typedef hello_world::remote_control::SwitchState RcSwitchState;

  typedef hello_world::module::Feed Feed;
  typedef hello_world::module::Fric Fric;

  typedef robot::GimbalChassisComm GimbalChassisComm;
  typedef robot::Chassis Chassis;
  typedef robot::Chassis::ChassisCmd ChassisCmd;
  typedef robot::Gimbal Gimbal;
  typedef robot::FricWorkingMode FricWorkingMode;

  enum MotorIdx : uint8_t {
    kMotorIdxFricLeft,  ///< 左摩擦轮电机下标
    kMotorIdxFricRight, ///< 右摩擦轮电机下标
    kMotorIdxFeed,      ///< 拨盘电机下标
    kMotorIdxYaw,       ///< YAW 轴电机下标
    kMotorIdxPitch,     ///< PITCH 轴电机下标
    kMotorNum,          ///< 电机数量
  };

public:
  Robot() {};
  ~Robot() {};

  // 状态机主要接口函数
  void update() override;

  void runOnDead() override;
  void runOnResurrection() override;
  void runOnWorking() override;
  void runAlways() override;

  void reset() override;

  void standby() override;

  void registerBuzzer(Buzzer *ptr);
  void registerImu(Imu *ptr);
  void registerLaser(Laser *ptr);
  void registerMotor(Motor *dev_ptr, uint8_t idx);
  void registerVision(Vision *dev_ptr);
  void registerRc(DT7 *ptr);

  void registerFeed(Feed *ptr);
  void registerFric(Fric *ptr);

  void registerChassis(Chassis *ptr);
  void registerGimbal(Gimbal *ptr);

  void registerGimbalChassisComm(GimbalChassisComm *dev_ptr);

private:
  //  数据更新和工作状态更新，由 update 函数调用
  void updateData();
  void updateImuData();
  void updateGimbalChassisCommData();
  void updateVisionData();
  void updateRcData();
  void updatePwrState();

  // 产生控制指令
  void genModulesCmd();
  void genModulesCmdFromRc();
  void genModulesCmdFromKb();
  void transmitFricStatus();

  // 设置通讯组件数据函数
  void setCommData();
  void setGimbalChassisCommData();
  void setVisionCommData();

  // 设置/重置数据函数
  void setManualCtrlSrc(ManualCtrlSrc src) {
    if (src != manual_ctrl_src_) {
      last_manual_ctrl_src_ = manual_ctrl_src_;
      manual_ctrl_src_ = src;
    }
  };
  void resetDataOnDead();
  void resetDataOnResurrection();

  // 发送通讯组件数据函数
  void sendCommData();
  void sendCanData();
  void sendFricsMotorData();
  void sendFeedMotorData();
  void sendGimbalMotorData();
  void sendGimbalChassisCommData();
  void sendUsartData();
  void sendVisionData();

  // IMU 数据在 update 函数中更新
  bool is_imu_caled_offset_ = false; ///< IMU 数据是否计算完零飘
  // UI Drawer数据在 update 函数中更新
  uint8_t refresh_ui_cnt_ = 0; ///< UI 刷新标志计数器
  // RC 数据在 update 函数中更新
  ManualCtrlSrc manual_ctrl_src_ = ManualCtrlSrc::kRc;      ///< 手动控制源
  ManualCtrlSrc last_manual_ctrl_src_ = ManualCtrlSrc::kRc; ///< 上一手动控制源

  float last_rev_chassis_tick_ = 0.0f; ///< 上一次底盘转向的时间戳
  float last_rev_gimbal_tick_ = 0.0f;  ///< 上一次云台转向的时间戳

  // 主要模块状态机组件指针
  Chassis *chassis_ptr_ = nullptr; ///< 底盘模块指针
  Gimbal *gimbal_ptr_ = nullptr;   ///< 云台模块指针
  Feed *feed_ptr_ = nullptr;       ///< 拨盘模块指针
  Fric *fric_ptr_ = nullptr;       ///< 摩擦轮模块指针

  // 无通信功能的组件指针
  Buzzer *buzzer_ptr_ = nullptr; ///< 蜂鸣器指针
  Imu *imu_ptr_ = nullptr;       ///< IMU 指针
  Laser *laser_ptr_ = nullptr;   ///< 红点激光指针

  // 只接收数据的组件指针
  DT7 *rc_ptr_ = nullptr; ///< DT7 指针 只接收数据

  // 只发送数据的组件指针
  Motor *motor_ptr_[kMotorNum] = {nullptr}; ///< 电机指针 只发送数据

  // 收发数据的组件指针
  GimbalChassisComm *gc_comm_ptr_ = nullptr; ///< 云台底盘通信模块指针 收发数据
  Vision *vision_ptr_ = nullptr;             ///< 视觉模块指针 收发数据
};
/* Exported variables
 * --------------------------------------------------------*/
/* Exported function prototypes
 * ----------------------------------------------*/
} // namespace robot
#endif /* ROBOT_MODULE_ROBOT_HPP_ */
