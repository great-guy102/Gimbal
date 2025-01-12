/**
 *******************************************************************************
 * @file      :ins_fsm.cpp
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
/* Includes ------------------------------------------------------------------*/
#include "ins_all.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
const robot::Gimbal::Config kGimbalConfig = {
    .sensitivity_yaw =
        360 / 1000.0 * PI / 180.0, ///< yaw角度灵敏度，单位 rad/ms
    .sensitivity_pitch =
        0.35 * 360 / 1000.0 * PI / 180.0, ///< pitch角度灵敏度，单位 rad/ms
    .max_pitch_ang = 0.50,                ///< 最大俯仰角度，单位 rad
    .min_pitch_ang = -0.30,               ///< 最小俯仰角度，单位 rad
    .max_pitch_torq = 0.98, ///< 云台水平时的重力矩，单位 N·m
    .pitch_center_offset =
        0.09, ///< 云台水平时，重心和pitch轴的连线与水平轴的夹角，单位 rad
};

const robot::Fric::Config kFricConfig = {
    .min_bullet_speed = 14.0f,     ///< 最小合理弹丸射速 m/s
    .max_bullet_speed = 16.5f,     ///< 最大合理弹丸射速 m/s
    .tgt_max_bullet_speed = 15.9f, ///< 最大目标弹丸速度 m/s
    .tgt_min_bullet_speed = 15.3f, ///< 最小目标弹丸速度 m/s
    .tgt_fric_spd_ref = 640.0f,    ///< 摩擦轮期望速度预设值 rad/s
    .tgt_fric_spd_ref_backward = -100.0f, ///< 摩擦轮反转目标速度

    .fric_stuck_curr_thre = 14.0f, ///< 用于判断摩擦轮堵转的电流阈值
    .fric_spd_delta_thre = 10.0f, ///< 用于判断摩擦轮速度保持恒定的阈值 (rad)
    .fric_spd_err_thre = 5.0f, ///< 用于判断摩擦轮速度跟上期望转速的阈值 (rad)
};

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
robot::Gimbal unique_gimbal = robot::Gimbal(kGimbalConfig);
robot::Fric unique_fric = robot::Fric(kFricConfig);
robot::Robot unique_robot = robot::Robot();
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
robot::Gimbal *GetGimbal() {
  static bool is_gimbal_initd = false;
  if (!is_gimbal_initd) {
    // 各组件指针
    // 无通信功能的组件指针
    unique_gimbal.registerPid(GetPidMotorYaw(), robot::Gimbal::kJointYaw);
    unique_gimbal.registerPid(GetPidMotorPitch(), robot::Gimbal::kJointPitch);

    unique_gimbal.registerTd(GetTdYaw(), robot::Gimbal::kJointYaw);
    unique_gimbal.registerTd(GetTdPitch(), robot::Gimbal::kJointPitch);

    // 只接收数据的组件指针
    unique_gimbal.registerImu(GetImu());

    //  接收、发送数据的组件指针
    unique_gimbal.registerMotor(GetMotorYaw(), robot::Gimbal::kJointYaw);
    unique_gimbal.registerMotor(GetMotorPitch(), robot::Gimbal::kJointPitch);

    is_gimbal_initd = true;
  }
  return &unique_gimbal;
};

robot::Fric *GetFric() {
  static bool is_fric_initd = false;
  if (!is_fric_initd) {
    // 各组件指针
    // 无通信功能的组件指针
    unique_fric.registerPid(GetPidMotorFricLeft(),
                            robot::Fric::kPidIdxFricLeft);
    unique_fric.registerPid(GetPidMotorFricRight(),
                            robot::Fric::kPidIdxFricRight);

    // 只接收数据的组件指针
    // 只发送数据的组件指针
    // 接收、发送数据的组件指针
    unique_fric.registerMotor(GetMotorFricLeft(),
                              robot::Fric::kMotorIdxFricLeft);
    unique_fric.registerMotor(GetMotorFricRight(),
                              robot::Fric::kMotorIdxFricRight);

    is_fric_initd = true;
  }
  return &unique_fric;
};
robot::Robot *GetRobot() {
  static bool is_robot_initd = false;
  if (!is_robot_initd) {
    // 各组件指针
    // 主要模块状态机组件指针
    unique_robot.registerGimbal(GetGimbal());

    unique_robot.registerFeed(GetFeed());
    unique_robot.registerFric(GetFric());

    // 无通信功能的组件指针
    unique_robot.registerBuzzer(GetBuzzer());
    unique_robot.registerLaser(GetLaser());
    unique_robot.registerImu(GetImu());

    // 有通信功能的组件指针

    hello_world::comm::CanTxMgr *can_tx_mgr_ptr;
    can_tx_mgr_ptr = GetCan2TxMgr();
    unique_robot.registerMotor(GetMotorFricLeft(),
                               robot::Robot::kMotorIdxFricLeft, can_tx_mgr_ptr);
    unique_robot.registerMotor(
        GetMotorFricRight(), robot::Robot::kMotorIdxFricRight, can_tx_mgr_ptr);
    unique_robot.registerMotor(GetMotorFeed(), robot::Robot::kMotorIdxFeed,
                               can_tx_mgr_ptr);
    unique_robot.registerMotor(GetMotorPitch(), robot::Robot::kMotorIdxPitch,
                               can_tx_mgr_ptr);

    can_tx_mgr_ptr = GetCan1TxMgr();
    unique_robot.registerMotor(GetMotorYaw(), robot::Robot::kMotorIdxYaw,
                               can_tx_mgr_ptr);
    unique_robot.registerGimbalChassisComm(GetGimbalChassisComm(),
                                           can_tx_mgr_ptr);

    hello_world::comm::UartTxMgr *uart_tx_mgr_ptr = GetVisionTxMgr();
    unique_robot.registerVision(GetVision(), uart_tx_mgr_ptr);

    is_robot_initd = true;
  }
  return &unique_robot;
};
/* Private function definitions ----------------------------------------------*/
