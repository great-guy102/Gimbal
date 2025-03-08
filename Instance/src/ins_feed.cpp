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
    .ang_ref_offset =
        0.0f, // 拨盘电机目标角度偏移量, >=0, 单位 rad(TODO 根据弹链长度调整)
    .ang_per_blt = PI / 5,    // 拨盘电机每个子弹的角度, >=0, 单位 rad
    .heat_per_blt = 10,       // 拨盘电机每个子弹的热量, >=0, 单位 J
    .stuck_curr_thre = 22.1f, // 电机卡住的电流阈值, >=0, 单位 A
    .resurrection_pos_err =
        5.0f / 180 * PI,             // 电机复活时的位置误差, >=0, 单位 rad
    .stuck_duration_thre = 200,      // 电机卡住的持续时间阈值, >=0, 单位 ms
    .hold_duration_thre = 100,       // 角度保持不变检测阈值时间, >=0, 单位 ms
    .default_trigger_interval = 200, // 默认触发间隔, >=0, 单位 ms
    .default_safe_num_blt = 3.0f, // 默认安全热量值对应的弹丸个数, >=0, 单位 个
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
