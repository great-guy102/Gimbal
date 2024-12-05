/** 
 *******************************************************************************
 * @file      :ins_motor.cpp
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
#include "ins_motor.hpp"
/* Private constants ---------------------------------------------------------*/

const hw_motor::OptionalParams kYawMotorParams = {
    .input_type = hw_motor::InputType::kTorq,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    /** 是否移除电机自带的减速器 */
    // .remove_build_in_reducer = false,
    /** 电机输出端实际角度与规定角度的差值 */
    .angle_offset = -1.5166986,
    /** 电机外置减速器的减速比（额外） */
    // .ex_redu_rat = 14,
};

const hw_motor::OptionalParams kPitchMotorParams= {
    .input_type = hw_motor::InputType::kTorq,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    // .dir = hw_motor::kDirFwd,  // 设置pitch轴电机低头角度低，抬头角度高
    /** 是否移除电机自带的减速器 */
    // .remove_build_in_reducer = false,
    /** 电机输出端实际角度与规定角度的差值 */
    .angle_offset = 0.364948988,
    /** 电机外置减速器的减速比（额外） */
    // .ex_redu_rat = 14,
};

const hw_motor::OptionalParams kFricMotorParams = {
    .input_type = hw_motor::InputType::kRaw,
    .angle_range = hw_motor::AngleRange::kNegInfToPosInf,
    /** 是否移除电机自带的减速器 */
    // .remove_build_in_reducer = true,
    /** 电机输出端实际角度与规定角度的差值 */
    .angle_offset = 0.0f,
};

const hw_motor::OptionalParams kFeedMotorParams = {
    // feed轮是2006电机
    .input_type = hw_motor::InputType::kTorq,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    /** 是否移除电机自带的减速器 */
    // .remove_build_in_reducer = true,
    /** 电机输出端实际角度与规定角度的差值 */
    .angle_offset = 0.0f,
    /** 电机外置减速器的减速比（额外） */
    // .ex_redu_rat = 51,
};

/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/

//yaw电机单例
static bool is_motor_yaw_inited = false;
hw_motor::DM_J4310 unique_motor_yaw;

//pitch电机单例
static bool is_motor_pitch_inited = false;
hw_motor::DM_J4310 unique_motor_pitch;

//摩擦轮电机单例
static bool is_motor_fric_left_inited = false;
hw_motor::M3508 unique_motor_fric_left;

static bool is_motor_fric_right_inited = false;
hw_motor::M3508 unique_motor_fric_right;

//feed轮电机单例
static bool is_motor_feed_inited = false;
hw_motor::M2006 unique_motor_feed;

/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

hw_motor::Motor* GetMotorYaw() { 
    if(!is_motor_yaw_inited){
        unique_motor_yaw = hw_motor::DM_J4310(0x01, kYawMotorParams);
        is_motor_yaw_inited = true;
    }
    return &unique_motor_yaw;
};

hw_motor::Motor* GetMotorPitch() { 
    if(!is_motor_pitch_inited){
        unique_motor_pitch = hw_motor::DM_J4310(0x02, kPitchMotorParams);
        is_motor_pitch_inited = true;
    }
    return &unique_motor_pitch;
};
hw_motor::Motor* GetMotorFricLeft() { 
    if(!is_motor_fric_left_inited){
        hw_motor::OptionalParams FricMotorParamsLeft = kFricMotorParams;
        FricMotorParamsLeft.dir = hw_motor::kDirFwd;
        unique_motor_fric_left = hw_motor::M3508(0x01, FricMotorParamsLeft);
        is_motor_fric_left_inited = true;
    }
    return &unique_motor_fric_left; 
};

hw_motor::Motor* GetMotorFricRight() { 
    if(!is_motor_fric_right_inited){
        hw_motor::OptionalParams FricMotorParamsRight = kFricMotorParams;
        FricMotorParamsRight.dir = hw_motor::kDirRev;
        unique_motor_fric_right = hw_motor::M3508(0x02, FricMotorParamsRight);
        is_motor_fric_right_inited = true;
    }
    return &unique_motor_fric_right; 
};

hw_motor::Motor* GetMotorFeed() { 
    if(!is_motor_feed_inited){
        unique_motor_feed = hw_motor::M2006(0x03, kFeedMotorParams);
        is_motor_feed_inited = true;
    }
    return &unique_motor_feed; 
};

/* Private function definitions ----------------------------------------------*/
