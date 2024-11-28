/**
 * @file      rfr_decoder.cpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-02-19
 * @brief
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 *
 * @copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All Rights Reserved.
 *
 * @attention
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2024-MM-DD | ZhouShichan | description |
 */
/* Define to prevent recursive inclusion -------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "rfr_decoder.hpp"

#include <cstring>

#include "rfr_crc.hpp"

namespace hello_world
{
namespace referee
{
using namespace internal;
/* Exported macro ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

RfrDecoder &RfrDecoder::operator=(const RfrDecoder &other)
{
  if (this == &other) {
    return *this;
  }

  rx_package_list_ = other.rx_package_list_;
  frame_ = other.frame_;
  data_index_ = other.data_index_;
  expect_length_ = other.expect_length_;
  rx_status_ = other.rx_status_;
  return *this;
}

RfrDecoder::RfrDecoder(RfrDecoder &&other)
{
  rx_package_list_ = std::move(other.rx_package_list_);
  frame_ = other.frame_;
  data_index_ = other.data_index_;
  expect_length_ = other.expect_length_;
  rx_status_ = other.rx_status_;
}

RfrDecoder &RfrDecoder::operator=(RfrDecoder &&other)
{
  if (this == &other) {
    return *this;
  }

  rx_package_list_ = std::move(other.rx_package_list_);
  frame_ = other.frame_;
  data_index_ = other.data_index_;
  expect_length_ = other.expect_length_;
  rx_status_ = other.rx_status_;
  return *this;
}

size_t RfrDecoder::decodeFrame(const uint8_t *data_ptr, size_t length)
{
  size_t n_decoded_pkgs = 0;
  for (size_t i = 0; i < length; i++) {
    if (processByte(data_ptr[i]) == RfrDcdResult::kRfrDcdOk) {
      n_decoded_pkgs++;
    }
  }
  return n_decoded_pkgs;
}

RfrDcdResult RfrDecoder::processByte(uint8_t byte)
{
  RfrDcdResult process_result = RfrDcdResult::kRfrDcdUndefinedErr;
  switch (rx_status_) {
    case kRxStatusWaitingHeaderSof:
      if (byte == kRefereeFrameHeaderSof) {
        frame_.raw[data_index_++] = byte;
        rx_status_ = kRxStatusHeader;
        process_result = RfrDcdResult::kRfrDcdGotHeaderSof;
      } else {
        restartDecodeFrame();
        process_result = RfrDcdResult::kRfrDcdWaitingHeaderSof;
      }
      break;
    case kRxStatusHeader:
      frame_.raw[data_index_++] = byte;
      if (data_index_ == sizeof(FrameHeader)) {
        if (VerifyCrc8CheckSum(frame_.raw, sizeof(FrameHeader))) {
          expect_length_ =
              frame_.decoded.header.data_length + sizeof(FrameHeader) +
              sizeof(CmdId) + sizeof(FrameTail);
          rx_status_ = kRxStatusCmdDataTail;
          process_result = RfrDcdResult::kRfrDcdGotHeader;
        } else {
          restartDecodeFrame();
          process_result = RfrDcdResult::kRfrDcdFailedCrc8;
        }
      } else {
        process_result = RfrDcdResult::kRfrDcdReceivingHeader;
      }

      if(data_index_==kRefereeMaxFrameLength){
        restartDecodeFrame();
      }
      break;
    case kRxStatusCmdDataTail:
      frame_.raw[data_index_++] = byte;
      if (data_index_ == expect_length_) {
        if (VerifyCrc16CheckSum(frame_.raw, expect_length_)) {
          if (decodeRxPackage(frame_.decoded.cmd_id, frame_.decoded.data)) {
            process_result = RfrDcdResult::kRfrDcdOk;
          } else {
            process_result = RfrDcdResult::kRfrDcdOkButNoPkg;
          }
        } else {
          process_result = RfrDcdResult::kRfrDcdFailedCrc16;
        }
        restartDecodeFrame();
      } else {
        process_result = RfrDcdResult::kRfrDcdWaitingTail;
      }
      break;
    default:
      restartDecodeFrame();
      break;
  }
  return process_result;
}
void RfrDecoder::restartDecodeFrame()
{
  memset(&frame_, 0, sizeof(frame_));
  data_index_ = 0;
  expect_length_ = 0;
  rx_status_ = kRxStatusWaitingHeaderSof;
}

bool RfrDecoder::decodeRxPackage(const CmdId &cmd_id, const uint8_t *data_ptr)
{
  bool process_result = false;
  for (auto rx_package : rx_package_list_) {
    process_result |= rx_package->decode(cmd_id, data_ptr);
  }
  return process_result;
}

void RfrDecoder::appendRxPackage(ProtocolRxPackage *rx_package_ptr)
{
  bool is_existed = false;
  for (auto rx_package : rx_package_list_) {
    if (rx_package == rx_package_ptr) {
      is_existed = true;
      break;
    }
  }
  if (!is_existed) {
    rx_package_list_.push_back(rx_package_ptr);
  }
}
/* Private function definitions ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world