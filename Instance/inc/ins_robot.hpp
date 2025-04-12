/**
 *******************************************************************************
 * @file      :ins_fsm.hpp
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
#ifndef INSTANCE_INS_FSM_HPP_
#define INSTANCE_INS_FSM_HPP_

/* Includes ------------------------------------------------------------------*/
#include "ins_feed.hpp"
#include "ins_fric.hpp"

#include "gimbal.hpp"
#include "robot.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

robot::Gimbal *GetGimbal();
robot::Robot *GetRobot();

hw_module::Feed *GetFeed();
hw_module::Fric *GetFric();
/* Exported function prototypes ----------------------------------------------*/

#endif /* INSTANCE_INS_FSM_HPP_ */
