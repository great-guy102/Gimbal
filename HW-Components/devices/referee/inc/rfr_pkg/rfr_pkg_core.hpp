/**
 * @file      rfr_pkg_core.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-25
 * @brief
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 *
 * @copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All
 * Rights Reserved.
 *
 * @attention
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2024-02-18 | ZhouShichan | 首次完成 |
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_CORE_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_CORE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "base.hpp"
#include "tick.hpp"

namespace hello_world
{
namespace referee
{
namespace tick = hello_world::tick;
/* Exported macro ------------------------------------------------------------*/
#define __REFEREE_PACKED __attribute__((packed))
#define FREQ2INTERVAL(freq) ((uint32_t)((freq) <= 1 ? 1000u : 1000u / (freq)))
/* Exported constants --------------------------------------------------------*/

const uint8_t kRefereeFrameHeaderSof = 0xA5;
const uint8_t kRefereeDataLength = 128u;
/* Exported types ------------------------------------------------------------*/

typedef uint8_t Crc8;
typedef uint16_t Crc16;
typedef uint16_t CmdId;
typedef uint16_t RfrId;
typedef uint16_t DataLength;
typedef uint32_t UpdateFreq;
typedef Crc16 FrameTail;

struct __REFEREE_PACKED FrameHeader {
  uint8_t sof;
  DataLength data_length;
  uint8_t seq;
  Crc8 crc8;
};

struct __REFEREE_PACKED CmdConfig {
  CmdId cmd_id;
  DataLength data_length;
};

#ifndef RFR_MAX_FRAME_LENGTH
const size_t kRefereeMaxFrameLength =
    kRefereeDataLength + sizeof(FrameHeader) + sizeof(FrameTail);
#else
const size_t kRefereeMaxFrameLength =
    RFR_MAX_FRAME_LENGTH >
            (kRefereeDataLength + sizeof(FrameHeader) + sizeof(FrameTail))
        ? RFR_MAX_FRAME_LENGTH
        : (kRefereeDataLength + sizeof(FrameHeader) + sizeof(FrameTail));
#endif

class ProtocolPackage : public MemMgr
{
 public:
  virtual CmdId getCmdId(void) const = 0;

  virtual DataLength getDataLength(void) const = 0;

 protected:
  uint32_t getNowTickMs(void) const { return tick::GetTickMs(); }
};

class ProtocolTxPackage : virtual public ProtocolPackage
{
 public:
  virtual uint32_t getMinTxIntervalMs(void) const = 0;

  virtual bool encode(uint8_t *data) = 0;

  uint32_t getLastEncodeTickMs(void) const { return last_encode_tick_; }

  virtual bool isTxIntervalSatisfied(void) const
  {
    return getNowTickMs() - last_encode_tick_ >= getMinTxIntervalMs();
  }

 protected:
  /*/< 记录上一次调用encode函数的时间，需要在 encode 函数中手动设置 */
  uint32_t last_encode_tick_ = 0;
};

class ProtocolRxPackage : virtual public ProtocolPackage
{
 public:
  virtual uint32_t getMaxRxIntervalMs(void) const = 0;

  uint32_t getLastDecodeTickMs(void) const { return last_decode_tick_; }

  /**
   * @brief 获取数据包是否已经更新
   *
   * 通过判断当前时间与上一次调用 decode
   * 函数的时间是否超过最大接收间隔以及用户是否处理过此包来判断数据包是否已经更新。由于
   * 用户处理数据包的频率往往比数据包更新频率高，因此在用户处理完数据但数据包并未超过最
   * 大接收间隔时，需要将数据包标记为已处理，以此来避免数据包被重复处理。
   * @return 数据包是否已经更新
   */
  bool isUpdated(void) const
  {
    uint32_t now_tick = getNowTickMs();
    return now_tick - last_decode_tick_ <= getMaxRxIntervalMs() &&
           is_handled_ == false;
  }

  /**
   * @brief 设置数据包已经被处理
   * @attention 需要用户自行在处理完数据包后设置为 true
   */
  void setHandled(void) { is_handled_ = true; }

  /**
   * @brief 获取数据包是否已经被处理
   * @return 数据包是否已经被处理
   */
  bool isHandled(void) const { return is_handled_; }

  /**
   * @brief 解码函数
   * @param cmd_id 命令码
   * @param data 数据
   * @return 解码是否成功
   * @note
   * 虚函数，需要在子类中实现。实际上，大部分数据包的解码方式都可以使用"命令码判断+位域
   * 结构体+memcpy"的方式来实现，但由于模板类无法在
   * Ozone 中正确显示，因此对每个数据包都单独编写子类。
   * @note 用户自行实现的 decode 函数需要在解码成功后手动设置 last_decode_tick_
   * 为当前时间，以及设置 is_handled_ 为 false
   */
  virtual bool decode(const CmdId &cmd_id, const uint8_t *data_ptr) = 0;

 protected:
  /** 上一次调用 decode 函数的时间 @details 需要在 decode 函数中手动设置，单位 ms */
  uint32_t last_decode_tick_ = 0;
  /** 是否已经被处理过 @details 需要在 decode 函数中手动设置为 false，表示数据包未被
   * 处理；需要用户自行在处理完数据包后设置为 true */
  bool is_handled_ = false;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_CORE_HPP_ */
