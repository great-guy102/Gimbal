/**
 *******************************************************************************
 * @file      :ins_filter.cpp
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
#include "ins_filter.hpp"
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
hw_filter::Td unique_td_yaw = hw_filter::Td(25, 0.001, 2 * PI, 1);
hw_filter::Td unique_td_pitch = hw_filter::Td(25, 0.001, 2 * PI, 1);
hw_filter::Ramp unique_ramp_pitch = hw_filter::Ramp(1.0f, 0.001f, 0.0f, 1);
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hw_filter::Td *GetTdYaw(void) { return &unique_td_yaw; };
hw_filter::Td *GetTdPitch(void) { return &unique_td_pitch; };
hw_filter::Ramp *GetRampPitch(void) { return &unique_ramp_pitch; };
/* Private function definitions ----------------------------------------------*/
