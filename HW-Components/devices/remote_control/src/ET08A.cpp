/**
 *******************************************************************************
 * @file      : ET08A.cpp
 * @brief     : WFLY 天地飞 ET08A 遥控器
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *  继承于 Sbus 类，注意事项请看 sbus.hpp 文件
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ET08A.hpp"

namespace hello_world
{
namespace remote_control
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

HW_OPTIMIZE_O2_START

ET08A::ET08A(const ET08AConfig& config) : Sbus(config.offline_tick_thres)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(IsSbusChValid(config.channel_map.J1), "Invalid J1 channel");
  HW_ASSERT(IsSbusChValid(config.channel_map.J2), "Invalid J2 channel");
  HW_ASSERT(IsSbusChValid(config.channel_map.J3), "Invalid J3 channel");
  HW_ASSERT(IsSbusChValid(config.channel_map.J4), "Invalid J4 channel");
  HW_ASSERT(IsSbusChValid(config.channel_map.SA), "Invalid SA channel");
  HW_ASSERT(IsSbusChValid(config.channel_map.SB), "Invalid SB channel");
  HW_ASSERT(IsSbusChValid(config.channel_map.SC), "Invalid SC channel");
  HW_ASSERT(IsSbusChValid(config.channel_map.SD), "Invalid SD channel");
  HW_ASSERT(IsSbusChValid(config.channel_map.LD), "Invalid LD channel");
  HW_ASSERT(IsSbusChValid(config.channel_map.RD), "Invalid RD channel");
#pragma endregion
  channel_map_ = config.channel_map;
}

ET08A& ET08A::operator=(const ET08A& other)
{
  if (this != &other) {
    Sbus::operator=(other);
    channel_map_ = other.channel_map_;
  }

  return *this;
}

ET08A::ET08A(ET08A&& other) : Sbus(std::move(other))
{
  channel_map_ = other.channel_map_;
}

ET08A& ET08A::operator=(ET08A&& other)
{
  if (this != &other) {
    Sbus::operator=(std::move(other));
    channel_map_ = other.channel_map_;
  }

  return *this;
}

bool ET08A::decode(size_t len, const uint8_t* data)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(data != nullptr, "Null pointer");
#pragma endregion

  bool ret = Sbus::decode(len, data);

  if (ret == true) {
    /* 遥杆旋钮解析 */
    SbusCh* ch_map1[6] = {
        &channel_map_.J1, &channel_map_.J2, &channel_map_.J3, &channel_map_.J4,
        &channel_map_.LD, &channel_map_.RD};
    float* val1[6] = {&J1_, &J2_, &J3_, &J4_, &LD_, &RD_};

    for (uint8_t i = 0; i < 6; i++) {
      if (*ch_map1[i] <= kSbusCh16) {
        *val1[i] = Ch2Pct(channels_[*ch_map1[i]]);
      }
    }

    /* 开关解析 */
    SbusCh* ch_map2[4] = {&channel_map_.SA, &channel_map_.SB, &channel_map_.SC,
                          &channel_map_.SD};
    SwitchState* val2[4] = {&SA_, &SB_, &SC_, &SD_};

    for (uint8_t i = 0; i < 4; i++) {
      if (*ch_map2[i] <= kSbusCh16) {
        *val2[i] = Ch2Sw(channels_[*ch_map2[i]]);
      }
    }
  }

  return ret;
}

void ET08A::init(const ET08AConfig& config)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(IsSbusChValid(config.channel_map.J1), "Invalid J1 channel");
  HW_ASSERT(IsSbusChValid(config.channel_map.J2), "Invalid J2 channel");
  HW_ASSERT(IsSbusChValid(config.channel_map.J3), "Invalid J3 channel");
  HW_ASSERT(IsSbusChValid(config.channel_map.J4), "Invalid J4 channel");
  HW_ASSERT(IsSbusChValid(config.channel_map.SA), "Invalid SA channel");
  HW_ASSERT(IsSbusChValid(config.channel_map.SB), "Invalid SB channel");
  HW_ASSERT(IsSbusChValid(config.channel_map.SC), "Invalid SC channel");
  HW_ASSERT(IsSbusChValid(config.channel_map.SD), "Invalid SD channel");
  HW_ASSERT(IsSbusChValid(config.channel_map.LD), "Invalid LD channel");
  HW_ASSERT(IsSbusChValid(config.channel_map.RD), "Invalid RD channel");
#pragma endregion

  Sbus::init(config.offline_tick_thres);

  channel_map_ = config.channel_map;
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace remote_control
}  // namespace hello_world