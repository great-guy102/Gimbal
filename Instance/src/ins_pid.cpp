/**
 *******************************************************************************
 * @file      :ins_pid.cpp
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
#include "ins_pid.hpp"

#include "motor.hpp"

/* Private constants ---------------------------------------------------------*/
const float kMaxPidOutYawAngle = 45.0f;
const float kMaxPidOutYawVel = 7.0f;
const float kMaxPidOutPitchAngle = 10.0f;
const float kMaxPidOutPitchVel = 7.0f;
const float kMaxPidOutFric = 16384.0f;
const float kMaxPidOutFeedAngle = 15000.0f;
const float kMaxPidOutFeedVel = 15000.0f;

const hw_pid::OutLimit kOutLimitYawAngle =
    hw_pid::OutLimit(true, -kMaxPidOutYawAngle, kMaxPidOutYawAngle);
const hw_pid::OutLimit kOutLimitYawVel =
    hw_pid::OutLimit(true, -kMaxPidOutYawVel, kMaxPidOutYawVel);
const hw_pid::OutLimit kOutLimitPitchAngle =
    hw_pid::OutLimit(true, -kMaxPidOutPitchAngle, kMaxPidOutPitchAngle);
const hw_pid::OutLimit kOutLimitPitchVel =
    hw_pid::OutLimit(true, -kMaxPidOutPitchVel, kMaxPidOutPitchVel);
const hw_pid::OutLimit kOutLimitFric =
    hw_pid::OutLimit(true, -kMaxPidOutFric, kMaxPidOutFric);
const hw_pid::OutLimit kOutLimitFeedAngle =
    hw_pid::OutLimit(true, -kMaxPidOutFeedAngle, kMaxPidOutFeedAngle);
const hw_pid::OutLimit kOutLimitFeedVel =
    hw_pid::OutLimit(true, -kMaxPidOutFeedVel, kMaxPidOutFeedVel);

const hw_pid::MultiNodesPid::ParamsList kPidParamsYaw = {
    {
        .auto_reset = true, ///< 是否自动清零
        .kp = 19.6f,
        .ki = 0.016f,
        .kd = 0.0f,
        // .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.1),
        .period_sub = hw_pid::PeriodSub(true, 2.0 * PI),
        .inte_changing_rate = hw_pid::InteChangingRate(true, 0.08f, 0.2f),
        .out_limit = kOutLimitYawAngle,
    },
    {
        .auto_reset = true, ///< 是否自动清零
        .kp = 1.6f,
        .ki = 0.0f,
        .kd = 0.0f,
        // .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.1),
        .out_limit = kOutLimitYawVel,
    },
};

const hw_pid::MultiNodesPid::ParamsList kPidParamsPitch = {
    {
        .auto_reset = true, ///< 是否自动清零
        .kp = 14.5f,        // 19.0f
        .ki = 0.009f,       // 0.005f
        .kd = 85.0f,        // 100.0f
        .max_interval_ms = 100,
        // .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.1),
        .period_sub = hw_pid::PeriodSub(true, 2.0 * PI),
        .inte_anti_windup = hw_pid::InteAntiWindup(true, -0.3f, 0.25f),
        .inte_changing_rate = hw_pid::InteChangingRate(true, 0.003f, 0.03f),
        .out_limit = kOutLimitPitchAngle,
    },
    {
        .auto_reset = true, ///< 是否自动清零
        .kp = 1.40f,
        .ki = 0.0f,
        .kd = 0.0f,
        // .setpoint_ramping   = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.2),
        .out_limit = kOutLimitPitchVel,
    }};

const hw_pid::MultiNodesPid::ParamsList kPidParamsFric = {
    {
        .auto_reset = true, ///< 是否自动清零
        .kp = 185.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .out_limit = kOutLimitFric,
    },
};

const hw_pid::MultiNodesPid::ParamsList kPidParamsFeed = {
    {
        .auto_reset = true, ///< 是否自动清零
        .kp = 17.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .period_sub = hw_pid::PeriodSub(true, 2.0 * PI),
        .out_limit = kOutLimitFeedAngle,
    },
    {
        .auto_reset = true, ///< 是否自动清零
        .kp = 1.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .out_limit = kOutLimitFeedVel,
    }};

const hw_pid::MultiNodesPid::Type kPidTypeCascade =
    hw_pid::MultiNodesPid::Type::kCascade;
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

hw_pid::MultiNodesPid unique_pid_yaw(kPidTypeCascade, kOutLimitYawVel,
                                     kPidParamsYaw);
hw_pid::MultiNodesPid unique_pid_pitch(kPidTypeCascade, kOutLimitPitchVel,
                                       kPidParamsPitch);
hw_pid::MultiNodesPid unique_pid_fric_left(kPidTypeCascade, kOutLimitFric,
                                           kPidParamsFric);
hw_pid::MultiNodesPid unique_pid_fric_right(kPidTypeCascade, kOutLimitFric,
                                            kPidParamsFric);
hw_pid::MultiNodesPid unique_pid_feed(kPidTypeCascade, kOutLimitFeedVel,
                                      kPidParamsFeed);

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hw_pid::MultiNodesPid *GetPidMotorYaw() { return &unique_pid_yaw; };
hw_pid::MultiNodesPid *GetPidMotorPitch() { return &unique_pid_pitch; };
hw_pid::MultiNodesPid *GetPidMotorFricLeft() { return &unique_pid_fric_left; };
hw_pid::MultiNodesPid *GetPidMotorFricRight() {
  return &unique_pid_fric_right;
};
hw_pid::MultiNodesPid *GetPidMotorFeed() { return &unique_pid_feed; };
/* Private function definitions ----------------------------------------------*/
