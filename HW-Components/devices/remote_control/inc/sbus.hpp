/**
 *******************************************************************************
 * @file      : Sbus.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_REMOTE_CONTROL_SBUS_HPP_
#define HW_COMPONENTS_DEVICES_REMOTE_CONTROL_SBUS_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "offline_checker.hpp"
#include "receiver.hpp"
#include "system.hpp"

namespace hello_world
{
namespace remote_control
{
/* Exported macro ------------------------------------------------------------*/

enum SbusCh : uint8_t {
  kSbusCh1 = 0u,
  kSbusCh2 = 1u,
  kSbusCh3 = 2u,
  kSbusCh4 = 3u,
  kSbusCh5 = 4u,
  kSbusCh6 = 5u,
  kSbusCh7 = 6u,
  kSbusCh8 = 7u,
  kSbusCh9 = 8u,
  kSbusCh10 = 9u,
  kSbusCh11 = 10u,
  kSbusCh12 = 11u,
  kSbusCh13 = 12u,
  kSbusCh14 = 13u,
  kSbusCh15 = 14u,
  kSbusCh16 = 15u,
  kSbusChInvalid = 0xFFu,
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

HW_OPTIMIZE_O2_START

class Sbus : public comm::Receiver
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  Sbus(void) = default;
  /**
   * @brief       构造函数
   * @param        offline_tick_thres: 离线阈值，单位：ms
   * @retval       None
   * @note        None
   */
  Sbus(uint32_t offline_tick_thres) : oc_(offline_tick_thres) {}
  Sbus(const Sbus &) = default;
  Sbus &operator=(const Sbus &other);
  Sbus(Sbus &&other);
  Sbus &operator=(Sbus &&other);

  virtual ~Sbus(void) = default;

  /* 重载方法 */

  virtual uint32_t rxId(void) const override
  {
    return 0u;
  }

  /**
   * @brief       解码 Sbus 协议数据
   * @param        len: 数据长度
   * @param        data: 数据指针
   * @retval       解码成功返回 true，否则返回 false
   * @note        None
   */
  virtual bool decode(size_t len, const uint8_t *data) override;

  virtual bool isUpdate(void) const override { return is_updated_; }

  virtual void clearUpdateFlag(void) override { is_updated_ = false; }

  /**
   * @brief       注册更新回调函数
   * @param        cb: 回调函数指针，在 decode 函数解码成功且 failsafe_ 与
   *               lost_frame_ 均为 false 后被调用，不使用时传入
   *               nullptr
   * @retval       None
   * @note        如注测封装好的看门狗刷新函数
   *
   */
  virtual void registerUpdateCallback(pUpdateCallback cb) override
  {
    update_cb_ = cb;
  }

  /* 配置方法 */

  /**
   * @brief       初始化，使用默认构造函数后请务必调用此函数
   * @param        offline_tick_thres: 离线阈值，单位：ms
   * @retval       None
   * @note        None
   */
  void init(uint32_t offline_tick_thres) { oc_.init(offline_tick_thres); }

  /* 数据修改与获取 */

  /**
   * @brief       获取通道值
   * @param        ch: 通道号
   * @retval       通道值
   * @note        None
   */
  uint16_t getChannel(SbusCh ch) const;

  bool failsafe(void) const { return failsafe_; }

  bool lost_frame(void) const { return lost_frame_; }

  bool isOffline(void) { return oc_.isOffline(); }

  static const uint8_t kNumChannels_ = 16;
  static const uint8_t kRcRxDataLen_ = 25;  ///* 遥控器接收数据长度

 protected:
  uint16_t channels_[kNumChannels_] = {0};
  bool failsafe_ = false;
  bool lost_frame_ = false;

  OfflineChecker oc_ = OfflineChecker(100u);

  bool is_updated_ = false;  ///* 是否有新数据更新
  pUpdateCallback update_cb_ = nullptr;

  /** 接收状态统计 */

  uint32_t decode_success_cnt_ = 0u;  ///* 解码成功次数
  uint32_t decode_fail_cnt_ = 0u;     ///* 解码失败次数
};
/* Exported function prototypes ----------------------------------------------*/

static inline bool IsSbusChValid(SbusCh ch)
{
  if (ch > kSbusCh16 && ch != kSbusChInvalid) {
    return false;
  } else {
    return true;
  }
}
HW_OPTIMIZE_O2_END
}  // namespace remote_control
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REMOTE_CONTROL_SBUS_HPP_ */
