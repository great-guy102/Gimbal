/**
 *******************************************************************************
 * @file      : Sbus.cpp
 * @brief     : 使用 Sbus 协议遥控器基类
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-07-25      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  1. 该类依赖串口接收管理器 UartRxMgr，使用前请确保 UartRxMgr 按要求配置于初始化，
 *  其中串口波特率设置为 100kbps，字长 9 Bits(include Parity)，偶校验，停止位 1，只
 *  接收。 串口接收管理器中 buf_len 设置为 26，max_process_data_len 设置为 25，
 *  eof_type 设置为 EofType::kIdle
 *  2. 由于接收机即使在遥控器断开时也会持续发送数据，因此只有在 failsafe_ 为 false 且
 *  lost_frame_ 为 false 时才会认为收到数据并调用回调函数
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "sbus.hpp"

#include "assert.hpp"

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

Sbus &Sbus::operator=(const Sbus &other)
{
  if (this != &other) {
    for (size_t i = 0; i < kNumChannels_; ++i) {
      channels_[i] = other.channels_[i];
    }

    failsafe_ = other.failsafe_;
    lost_frame_ = other.lost_frame_;

    oc_ = other.oc_;

    is_updated_ = other.is_updated_;
    update_cb_ = other.update_cb_;

    decode_success_cnt_ = other.decode_success_cnt_;
    decode_fail_cnt_ = other.decode_fail_cnt_;
  }
  return *this;
}

Sbus::Sbus(Sbus &&other)
{
  for (size_t i = 0; i < kNumChannels_; ++i) {
    channels_[i] = other.channels_[i];
  }

  failsafe_ = other.failsafe_;
  lost_frame_ = other.lost_frame_;

  oc_ = std::move(other.oc_);

  is_updated_ = other.is_updated_;
  update_cb_ = other.update_cb_;

  decode_success_cnt_ = other.decode_success_cnt_;
  decode_fail_cnt_ = other.decode_fail_cnt_;

  other.update_cb_ = nullptr;
}

Sbus &Sbus::operator=(Sbus &&other)
{
  if (this != &other) {
    for (size_t i = 0; i < kNumChannels_; ++i) {
      channels_[i] = other.channels_[i];
    }

    failsafe_ = other.failsafe_;
    lost_frame_ = other.lost_frame_;

    oc_ = std::move(other.oc_);

    is_updated_ = other.is_updated_;
    update_cb_ = other.update_cb_;

    decode_success_cnt_ = other.decode_success_cnt_;
    decode_fail_cnt_ = other.decode_fail_cnt_;

    other.update_cb_ = nullptr;
  }
  return *this;
}

bool Sbus::decode(size_t len, const uint8_t *data)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(data != nullptr, "Error data pointer");
#pragma endregion

  if (len != kRcRxDataLen_) {
    return false;
  }

  /* 校验 */
  if (data[0] != 0x0F || data[24] != 0x00) {
    decode_fail_cnt_++;
    return false;
  }

  /* 解码 */
  channels_[0] = ((data[1] | data[2] << 8) & 0x07FF);
  channels_[1] = ((data[2] >> 3 | data[3] << 5) & 0x07FF);
  channels_[2] =
      ((data[3] >> 6 | data[4] << 2 | data[5] << 10) & 0x07FF);
  channels_[3] = ((data[5] >> 1 | data[6] << 7) & 0x07FF);
  channels_[4] = ((data[6] >> 4 | data[7] << 4) & 0x07FF);
  channels_[5] =
      ((data[7] >> 7 | data[8] << 1 | data[9] << 9) & 0x07FF);
  channels_[6] = ((data[9] >> 2 | data[10] << 6) & 0x07FF);
  channels_[7] = ((data[10] >> 5 | data[11] << 3) & 0x07FF);
  channels_[8] = ((data[12] | data[13] << 8) & 0x07FF);
  channels_[9] = ((data[13] >> 3 | data[14] << 5) & 0x07FF);
  channels_[10] =
      ((data[14] >> 6 | data[15] << 2 | data[16] << 10) & 0x07FF);
  channels_[11] = ((data[16] >> 1 | data[17] << 7) & 0x07FF);
  channels_[12] = ((data[17] >> 4 | data[18] << 4) & 0x07FF);
  channels_[13] =
      ((data[18] >> 7 | data[19] << 1 | data[20] << 9) & 0x07FF);
  channels_[14] = ((data[20] >> 2 | data[21] << 6) & 0x07FF);
  channels_[15] = ((data[21] >> 5 | data[22] << 3) & 0x07FF);

  failsafe_ = (data[23] & 0x08) ? true : false;
  lost_frame_ = (data[23] & 0x04) ? true : false;

  decode_success_cnt_++;

  if (!failsafe_ && !lost_frame_) {
    oc_.update();

    is_updated_ = true;
    if (update_cb_) {
      update_cb_();
    }
  }

  return true;
}

uint16_t Sbus::getChannel(SbusCh ch) const
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(ch <= kSbusCh16, "Error channel number");
#pragma endregion

  return channels_[ch];
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace remote_control
}  // namespace hello_world
