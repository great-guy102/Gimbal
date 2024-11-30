/** 
 *******************************************************************************
 * @file      : ins_imu.cpp
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
#include "ins_imu.hpp"
/* Private constants ---------------------------------------------------------*/
const float kImuRotMatFlatten[9] = {-1, 0, 0, 0, -1, 0, 0, 0, 1};
const robot::Imu::Config kImuInitParams = {
    .rot_mat_ptr = kImuRotMatFlatten,
};
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
robot::Imu *GetImu(void) { 
    static robot::Imu unique_imu = robot::Imu(kImuInitParams);
    return &unique_imu; 
};

// // ADD: 为解决硬件初始化未完成而导致的进入断言，增加BMI088的参数设置调用函数
// void ImuInitBmi088(void) { robot::Imu bmi088ConfigInit(kImuInitParams); };
/* Private function definitions ----------------------------------------------*/
