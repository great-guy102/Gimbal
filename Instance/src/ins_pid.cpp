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
const float kMaxPidOutYawAngle = 10.0f;
const float kMaxPidOutYawVel = 7.0f;
const float kMaxPidOutPitchAngle = 10.0f;
const float kMaxPidOutPitchVel = 7.0f;
const float kMaxPidOutFric = 16384.0f;
const float kMaxPidOutFeedAngle = 15000.0f;
const float kMaxPidOutFeedVel = 15000.0f; 


const hw_pid::OutLimit kOutLimitYawAngle = hw_pid::OutLimit(true, -kMaxPidOutYawAngle, kMaxPidOutYawAngle);
const hw_pid::OutLimit kOutLimitYawVel = hw_pid::OutLimit(true, -kMaxPidOutYawVel, kMaxPidOutYawVel);
const hw_pid::OutLimit kOutLimitPitchAngle = hw_pid::OutLimit(true, -kMaxPidOutPitchAngle, kMaxPidOutPitchAngle);
const hw_pid::OutLimit kOutLimitPitchVel = hw_pid::OutLimit(true, -kMaxPidOutPitchVel, kMaxPidOutPitchVel);
const hw_pid::OutLimit kOutLimitFric = hw_pid::OutLimit(true, -kMaxPidOutFric, kMaxPidOutFric);
const hw_pid::OutLimit kOutLimitFeedAngle = hw_pid::OutLimit(true, -kMaxPidOutFeedAngle, kMaxPidOutFeedAngle);
const hw_pid::OutLimit kOutLimitFeedVel = hw_pid::OutLimit(true, -kMaxPidOutFeedVel, kMaxPidOutFeedVel);


const hw_pid::MultiNodesPid::ParamsList kPidParamsYaw = {
    {
        .auto_reset = true,  ///< 是否自动清零
        .kp = 32.0f,
        .ki = 0.0f,
        .kd = 100.0f,
        .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.1),
        .period_sub = hw_pid::PeriodSub(true, 2.0 * PI),
        .inte_separation = hw_pid::InteSeparation(true, -0.01f, 0.01f),
        .diff_previous = hw_pid::DiffPrevious(false, 0.5f),
        .out_limit = kOutLimitYawAngle,  ///< 输出限制 @see OutLimit
    },
    {
        .auto_reset = true,  ///< 是否自动清零
        .kp = 1.7f,
        .ki = 0.000f,
        .kd = 0.0f,
        .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.1),
        .inte_anti_windup = hw_pid::InteAntiWindup(true, -3.0, 3.0),
        .inte_changing_rate = hw_pid::InteChangingRate(false, 0.1f, 0.1f),
        .inte_separation = hw_pid::InteSeparation(true, -1.0f, 1.0f),
        .out_limit = kOutLimitYawVel,  ///< 输出限制 @see OutLimit
    },
};

const hw_pid::MultiNodesPid::ParamsList kPidParamsPitch = {
    {
        .auto_reset = true,  ///< 是否自动清零
        .kp = 28.0f,
        .ki = 0.0f,
        .kd = 0.01f,
        .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.1),
        .period_sub = hw_pid::PeriodSub(true, 2.0 * PI),
        .out_limit = kOutLimitPitchAngle,  ///< 输出限制 @see OutLimit
    },
    {
        .auto_reset = true,  ///< 是否自动清零
        .kp = 1.2f,
        .ki = 0.004f,
        .kd = 0.0f,
        .setpoint_ramping   = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.2),
        .inte_anti_windup = hw_pid::InteAntiWindup(true, -1.5, 1.5),
        .out_limit = kOutLimitPitchVel,  ///< 输出限制 @see OutLimit
    }
};

const hw_pid::MultiNodesPid::ParamsList kPidParamsFric = {
    {
        .auto_reset = true,  ///< 是否自动清零
        .kp = 3300.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .out_limit = kOutLimitFric,
    },
};

const hw_pid::MultiNodesPid::ParamsList kPidParamsFeed = {
   {
        .auto_reset = true,  ///< 是否自动清零
        .kp = 17.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .period_sub = hw_pid::PeriodSub(true, 2.0 * PI),
        .out_limit = kOutLimitFeedAngle,  ///< 输出限制 @see OutLimit
    },
    {
        .auto_reset = true,  ///< 是否自动清零
        .kp = 1.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .out_limit = kOutLimitFeedVel,  ///< 输出限制 @see OutLimit
    }
};

const hw_pid::MultiNodesPid::Type kPidTypeCascade = hw_pid::MultiNodesPid::Type::kCascade;
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

hw_pid::MultiNodesPid unique_pid_yaw(kPidTypeCascade, kOutLimitYawVel, kPidParamsYaw);
hw_pid::MultiNodesPid unique_pid_pitch(kPidTypeCascade, kOutLimitPitchVel, kPidParamsPitch);
hw_pid::MultiNodesPid unique_pid_fric_left(kPidTypeCascade, kOutLimitFric, kPidParamsFric);
hw_pid::MultiNodesPid unique_pid_fric_right(kPidTypeCascade, kOutLimitFric, kPidParamsFric);
hw_pid::MultiNodesPid unique_pid_feed(kPidTypeCascade, kOutLimitFeedVel, kPidParamsFeed);

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hw_pid::MultiNodesPid* GetPidMotorYaw() { return &unique_pid_yaw; };
hw_pid::MultiNodesPid* GetPidMotorPitch() { return &unique_pid_pitch; };
hw_pid::MultiNodesPid* GetPidMotorFricLeft() { return &unique_pid_fric_left; };
hw_pid::MultiNodesPid* GetPidMotorFricRight() { return &unique_pid_fric_right; };
hw_pid::MultiNodesPid* GetPidMotorFeed() { return &unique_pid_feed; };
/* Private function definitions ----------------------------------------------*/
