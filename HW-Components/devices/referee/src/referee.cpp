/**
 *******************************************************************************
 * @file      : referee.cpp
 * @brief     : 裁判系统收发管理类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2024-01-25      ZhouShichan     1. 未测试版本
 *  V1.0.0      2024-07-13      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  该类依赖串口接收管理器 UartRxMgr 与串口发送管理器 UartTxMgr，使用前请确保
 *  UartRxMgr 与 UartTxMgr 按要求配置于初始化，其中串口波特率设置为 115200，字长
 *  8 Bits(include Parity)，无校验，停止位 1。
 *  串口接收管理器中 buf_len 设置为 32，max_process_data_len 设置为 32，eof_type 设
 *  置为 EofType::kManual。
 *  串口发送管理器中 buf_len 建议设置不小于
 *  hello_world::referee::kRefereeMaxFrameLength。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "referee.hpp"

#include <cstring>

#include "rfr_crc.hpp"

namespace hello_world
{
namespace referee
{
using namespace internal;
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

Referee &Referee::operator=(const Referee &other)
{
  if (this == &other) {
    return *this;
  }

  rx_frame_ = other.rx_frame_;
  rx_pkg_list_ = other.rx_pkg_list_;
  rx_status_ = other.rx_status_;
  rx_result_ = other.rx_result_;
  is_update_ = other.is_update_;
  update_cb_ = other.update_cb_;
  rx_data_index_ = other.rx_data_index_;
  rx_expect_length_ = other.rx_expect_length_;
  rx_n_pkgs_ = other.rx_n_pkgs_;
  rx_err_cnt_ = other.rx_err_cnt_;
  offline_tick_thres_ = other.offline_tick_thres_;
  oc_ = other.oc_;

  tx_frame_ = other.tx_frame_;
  tx_pkg_ready_ = other.tx_pkg_ready_;

  return *this;
}

Referee::Referee(Referee &&other)
{
  rx_frame_ = other.rx_frame_;
  rx_pkg_list_ = std::move(other.rx_pkg_list_);
  rx_status_ = other.rx_status_;
  rx_result_ = other.rx_result_;
  is_update_ = other.is_update_;
  update_cb_ = other.update_cb_;
  rx_data_index_ = other.rx_data_index_;
  rx_expect_length_ = other.rx_expect_length_;
  rx_n_pkgs_ = other.rx_n_pkgs_;
  rx_err_cnt_ = other.rx_err_cnt_;
  offline_tick_thres_ = other.offline_tick_thres_;
  oc_ = std::move(other.oc_);

  tx_frame_ = other.tx_frame_;
  tx_pkg_ready_ = other.tx_pkg_ready_;

  other.update_cb_ = nullptr;
}

Referee &Referee::operator=(Referee &&other)
{
  if (this == &other) {
    return *this;
  }

  rx_frame_ = other.rx_frame_;
  rx_pkg_list_ = std::move(other.rx_pkg_list_);
  rx_status_ = other.rx_status_;
  rx_result_ = other.rx_result_;
  is_update_ = other.is_update_;
  update_cb_ = other.update_cb_;
  rx_data_index_ = other.rx_data_index_;
  rx_expect_length_ = other.rx_expect_length_;
  rx_n_pkgs_ = other.rx_n_pkgs_;
  rx_err_cnt_ = other.rx_err_cnt_;
  offline_tick_thres_ = other.offline_tick_thres_;
  oc_ = std::move(other.oc_);

  tx_frame_ = other.tx_frame_;
  tx_pkg_ready_ = other.tx_pkg_ready_;

  other.update_cb_ = nullptr;

  return *this;
}

bool Referee::decode(size_t len, const uint8_t *data)
{
  rx_n_pkgs_ = 0;
  if (data == nullptr || len == 0) {
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    rx_result_ = processByte(data[i]);  // 逐字节处理
    if (isRxOk()) {
      rx_n_pkgs_++;
    } else if (isRxErr()) {
      rx_err_cnt_++;
    }
  }

  if (rx_n_pkgs_ != 0) {
    oc_.update();

    is_update_ = true;
    if (update_cb_) {
      update_cb_();
    }
  }

  return rx_n_pkgs_ > 0;
}

bool Referee::encode(size_t &len, uint8_t *data)
{
  if (data == nullptr || !tx_pkg_ready_) {
    encode_fail_cnt_++;
    return false;
  }

  if (len < tx_frame_len_) {
    encode_fail_cnt_++;
    return false;
  }

  memcpy(data, tx_frame_.raw, tx_frame_len_);
  len = tx_frame_len_;
  encode_success_cnt_++;

  return true;
}

void Referee::resetDecode(void)
{
  rx_status_ = RxStatus::kWaitingHeaderSof;
  rx_result_ = RxResult::kErrNoDataInput;

  rx_data_index_ = 0;
  rx_expect_length_ = 0;

  memset(&rx_frame_, 0, sizeof(rx_frame_));
}

void Referee::appendRxPkg(RxPkg *rx_pkg_ptr)
{
  if (rx_pkg_ptr == nullptr) {
    return;
  }

  /* 避免重复添加 */
  for (auto &it : rx_pkg_list_) {
    if (it == rx_pkg_ptr) {
      return;
    }
  }
  rx_pkg_list_.push_back(rx_pkg_ptr);

  /* 更新掉线阈值 */
  if (offline_tick_thres_ == 0) {
    uint32_t offline_threshold = rx_pkg_ptr->getMaxRxIntervalMs() * 3;
    if (offline_threshold < oc_.get_offline_tick_thres()) {
      oc_.set_offline_tick_thres(offline_threshold);
    }
  }
}

void Referee::clearRxPkgList(void)
{
  if (rx_pkg_list_.empty()) {
    return;
  }

  rx_pkg_list_.clear();

  /* 重置离线检测时间阈值 */
  if (offline_tick_thres_ != 0) {
    oc_.set_offline_tick_thres(offline_tick_thres_);
  } else {
    oc_.set_offline_tick_thres(1000);
  }
}

void Referee::eraseRxPkg(RxPkg *rx_pkg_ptr)
{
  if (rx_pkg_ptr == nullptr || rx_pkg_list_.empty()) {
    return;
  }

  /* 重置离线检测时间阈值 */
  uint32_t offline_threshold = oc_.get_offline_tick_thres();
  for (auto it = rx_pkg_list_.begin(); it != rx_pkg_list_.end(); ++it) {
    if (*it == rx_pkg_ptr) {
      rx_pkg_list_.erase(it);
    } else {
      uint32_t tmp_threshold = (*it)->getMaxRxIntervalMs() * 3;
      if (tmp_threshold < offline_threshold) {
        offline_threshold = tmp_threshold;
      }
    }
  }

  if (offline_tick_thres_ == 0) {
    oc_.set_offline_tick_thres(offline_threshold);
  }
}

bool Referee::setTxPkg(ProtocolTxPackage *tx_pkg_ptr)
{
  if (tx_pkg_ptr == nullptr) {
    return false;
  }

  /* 判断是否满足发送标准 */
  if (tx_pkg_ready_ || !tx_pkg_ptr->isTxIntervalSatisfied()) {
    return false;
  }

  auto &tx_msg = tx_frame_.msg;
  tx_msg.header.sof = kRefereeFrameHeaderSof;
  tx_msg.header.data_length = tx_pkg_ptr->getDataLength();
  tx_msg.header.seq = tx_msg.header.seq == 0xFF ? 0 : tx_msg.header.seq + 1;
  if (!SetEndCrc8CheckSum(tx_frame_.raw, sizeof(FrameHeader))) {
    encode_fail_cnt_++;
    return false;
  }
  tx_msg.cmd_id = tx_pkg_ptr->getCmdId();

  if (!tx_pkg_ptr->encode(tx_msg.data)) {
    encode_fail_cnt_++;
    return false;
  }

  tx_frame_len_ = sizeof(FrameHeader) + sizeof(CmdId) +
                  tx_pkg_ptr->getDataLength() + sizeof(Crc16);
  if (!SetEndCrc16CheckSum(tx_frame_.raw, tx_frame_len_)) {
    encode_fail_cnt_++;
    tx_pkg_ready_ = false;
    return false;
  }

  tx_pkg_ready_ = true;
  return true;
}

Referee::RxResult Referee::processByte(uint8_t byte)
{
  RxResult result = RxResult::kErrUndefined;
  switch (rx_status_) {
    case RxStatus::kWaitingHeaderSof:
      if (byte == kRefereeFrameHeaderSof) {
        resetDecode();
        rx_frame_.raw[rx_data_index_++] = byte;
        rx_status_ = RxStatus::kWaitingHeaderCplt;
        result = RxResult::kHandlingHeader;
      } else {
        resetDecode();
        result = RxResult::kHandlingWaitSof;
      }
      break;
    case RxStatus::kWaitingHeaderCplt:
      result = RxResult::kHandlingHeader;
      rx_frame_.raw[rx_data_index_++] = byte;
      if (rx_data_index_ == sizeof(FrameHeader)) {
        if (VerifyCrc8CheckSum(rx_frame_.raw, sizeof(FrameHeader))) {
          rx_expect_length_ =
              rx_frame_.msg.header.data_length + sizeof(FrameHeader) +
              sizeof(CmdId) + sizeof(FrameTail);
          rx_status_ = RxStatus::kWaitingTailCplt;
          result = RxResult::kHandlingWaitTail;
        } else {
          resetDecode();
          result = RxResult::kErrFailedCrc8;
        }
      }

      if (rx_data_index_ > sizeof(FrameHeader)) {
        resetDecode();
        result = RxResult::kErrTooLongHeader;
      }
      break;
    case RxStatus::kWaitingTailCplt:
      result = RxResult::kHandlingWaitTail;
      rx_frame_.raw[rx_data_index_++] = byte;
      if (rx_data_index_ >= kRefereeMaxFrameLength) {
        resetDecode();
        result = RxResult::kErrTooLongData;
      }

      if (rx_data_index_ == rx_expect_length_) {
        if (VerifyCrc16CheckSum(rx_frame_.raw, rx_expect_length_)) {
          if (decodeRxPackage(rx_frame_.msg.cmd_id, rx_frame_.msg.data)) {
            result = RxResult::kOkWithPkg;
          } else {
            result = RxResult::kOkWithoutPkg;
          }
        } else {
          result = RxResult::kErrFailedCrc16;
        }
        resetDecode();
      }
      break;
    default:
      resetDecode();
      break;
  }

  return result;
}

bool Referee::decodeRxPackage(const CmdId &cmd_id, const uint8_t *data_ptr)
{
  bool result = false;
  for (auto &it : rx_pkg_list_) {
    result = result || it->decode(cmd_id, data_ptr);
  }
  return result;
}
/* Private function definitions ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world