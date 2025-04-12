/**
 *******************************************************************************
 * @file      :ins_feed.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.1.0      2025-1-11       Wpy             Not yet
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team,Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/

#include "ins_feed.hpp"
#include "ins_motor.hpp"
#include "ins_pid.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
hw_module::Feed::Config kFeedConfig = {
    .ang_ref_offset = 0.0f,
    .ang_per_blt = PI / 5.0f,
    .heat_per_blt = 10,
    .resurrection_pos_err = 5.0f / 180 * PI,
    .hold_duration_thre = 100,
    .default_trigger_interval = 200,
    .default_safe_num_blt = 1.5f,
    .stuck_detection =
        {.detection_by =
             hello_world::module::feed_impl::FeedStuckDetectionBy::kAngle,
         .ang_diff_thre = 0.65f,
         .curr_thre = 8.0f,
         .time_thre = 100},
    .stuck_handle =
        {.method =
             hello_world::module::feed_impl::FeedStuckBackMethod::kToLastAng,
         .back_ang = 0.0f},
    .stuck_prevent = {.enabled = false, .back_ang = 0.0f},
};

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
hw_module::Feed unique_feed = hw_module::Feed(kFeedConfig);
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
hw_module::Feed *GetFeed() {
  static bool is_feed_created = false;
  if (!is_feed_created) {
    unique_feed.registerPidFeed(GetPidMotorFeed()); // 注册MultiNodesPID指针
    unique_feed.registerMotorFeed(GetMotorFeed());  // 注册电机指针
    is_feed_created = true;
  }
  return &unique_feed;
};
