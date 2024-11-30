/** 
 *******************************************************************************
 * @file      : ins_fsm.hpp
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
#include "gimbal.hpp"
#include "robot.hpp"
#include "feed.hpp"
#include "fric.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

robot::Gimbal* GetGimbal();
robot::Feed* GetFeed();
robot::Fric* GetFric();
robot::Robot* GetRobot();

/* Exported function prototypes ----------------------------------------------*/

#endif /* INSTANCE_INS_FSM_HPP_ */
