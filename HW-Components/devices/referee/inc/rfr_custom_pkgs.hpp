/**
 *******************************************************************************
 * @file      : rfr_custom_pkgs.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_CUSTOM_PKGS_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_CUSTOM_PKGS_HPP_

/* Includes ------------------------------------------------------------------*/
#include "rfr_pkg/rfr_id.hpp"
#include "rfr_pkg/rfr_pkg_0x0301_inter_among_robots.hpp"
#include "rfr_pkg/rfr_pkg_core.hpp"

namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class CustomProtocolPackage : public ProtocolRxPackage,
                              public InterAmongRobotsPackage
{
 public:
  CustomProtocolPackage(void) = default;
  CustomProtocolPackage(const CustomProtocolPackage &) = default;
  CustomProtocolPackage &operator=(const CustomProtocolPackage &) = default;
  CustomProtocolPackage(CustomProtocolPackage &&) = default;
  CustomProtocolPackage &operator=(CustomProtocolPackage &&) = default;

  virtual ~CustomProtocolPackage(void) = default;

  virtual uint32_t getMaxRxIntervalMs(void) const = 0;

  /**
   * @brief 解码函数
   * @param cmd_id 命令码
   * @param data 数据
   * @return 解码是否成功
   * @note 用户自行实现的 decode 函数需要在解码成功后手动设置 last_decode_tick_
   * 为当前时间，以及设置 is_handled_ 为 false
   */
  virtual bool decode(const CmdId &cmd_id, const uint8_t *data_ptr)
  {
    if (cmd_id == getCmdId()) {
      InterConfig inter_config;
      memcpy(&inter_config, data_ptr, sizeof(InterConfig));
      if (inter_config.data_cmd_id == getInterCmdId() &&
          sender_id_ == inter_config.receiver_id) {
        decodeInterData(data_ptr + sizeof(InterConfig));
        last_decode_tick_ = getNowTickMs();
        is_handled_ = false;
        return true;
      }
    }
    return false;
  }

  virtual CmdId getInterCmdId(void) const = 0;

  virtual DataLength getInterDataLength(void) const = 0;

 protected:
  virtual bool checkSenderId(RfrId id) const { return ids::IsRobotId(id); }

  virtual bool checkReceiverId(RfrId id) const
  {
    /* TODO(ZSC): 这里需要判断机器人间通讯 id 是否合法（如哨兵的通讯限制），暂时只做id
       类型检查和颜色检查 */
    return ids::IsRobotId(id) && ids::IsSameColer(id, sender_id_);
  }

  /*/< 编码交互数据包的子内容，需要在子类中实现。外部则调用此类的 encode() 函数 */
  virtual void encodeInterData(uint8_t *data) = 0;

  /*/< 解码交互数据包的子内容，需要在子类中实现。外部则调用此类的 decode() 函数 */
  virtual bool decodeInterData(const uint8_t *data_ptr) = 0;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_CUSTOM_PKGS_HPP_ */
