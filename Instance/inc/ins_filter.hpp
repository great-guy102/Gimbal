/**
 *******************************************************************************
 * @file      :ins_filter.hpp
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
#ifndef INSTANCE_INS_FILTER_HPP_
#define INSTANCE_INS_FILTER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "base.hpp"
#include "filter.hpp"
#include "ramp.hpp"

namespace hw_filter = hello_world::filter;
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
hw_filter::Td *GetTdYaw(void);
hw_filter::Td *GetTdPitch(void);
hw_filter::Ramp *GetRampPitch(void);
#endif /* INSTANCE_INS_FILTER_HPP_ */
