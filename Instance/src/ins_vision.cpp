/**
 *******************************************************************************
 * @file      :ins_vision.cpp
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
#include "ins_vision.hpp"
/* Private constants ---------------------------------------------------------*/
const hw_vision::Vision::Config vision_cfg = {
        .default_blt_spd = 22.0f,
        .blt_spd_filter_beta = 0.9f,
        .offline_thres = 100u,
        .hfov = 0.52f,
        .vfov = 0.52f,
    };
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
hw_vision::Vision unique_vision = hw_vision::Vision(vision_cfg);
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hw_vision::Vision *GetVision() { return &unique_vision; };
/* Private function definitions ----------------------------------------------*/
