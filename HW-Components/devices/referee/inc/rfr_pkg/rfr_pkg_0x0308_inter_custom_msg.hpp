/**
 * @file      rfr_pkg_0x0308_inter_custom_msg.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-02-18
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0308_INTER_CUSTOM_MSG_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0308_INTER_CUSTOM_MSG_HPP_

/* Includes ------------------------------------------------------------------*/
#include "rfr_id.hpp"
#include "rfr_pkg_core.hpp"

namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/**
 * @struct InterCustomMsgData
 * @brief 机器人间自定义消息数据
 */
struct __REFEREE_PACKED InterCustomMsgData {
  uint16_t sender_id;     ///< 发送者的 ID @note 需要校验发送者的 ID 正确性
  /*/< 接收者的 ID @note 需要校验接收者的 ID 正确性，仅支持发送己方选手端 */
  uint16_t receiver_id;
  /** 字符消息 @note 以 utf-16 格式编码发送，支持显示中文。编码发送时请注意数据的大小
   * 端问题 */
  uint8_t user_data[30];
};
static_assert(sizeof(InterCustomMsgData) == 34, "InterCustomMsgData size error");
/**
 * @class InterCustomMsgPackage
 * @brief 选手端小地图接收的机器人间自定义消息数据包
 *
 * 己方机器人可通过常规链路向己方任意选手端发送自定义的消息，该消息会在己方选手端特定位
 * 置显示。
 *
 * 数据说明：
 *
 * - 命令码：0x0308
 * - 数据长度：34
 * - 发送频率：频率上限为 3Hz
 * - 发送方/接收方：己方机器人->己方选手端
 * - 所属数据链路：常规链路
 */
class InterCustomMsgPackage : public ProtocolTxPackage
{
 public:
  typedef InterCustomMsgData Data;
  typedef ids::TeamColor TeamColor;

  virtual CmdId getCmdId(void) const override { return 0x0308; }
  virtual DataLength getDataLength(void) const override { return sizeof(Data); }
  virtual uint32_t getMinTxIntervalMs() const override
  {
    return FREQ2INTERVAL(3);
  }

  const Data &getData(void) const { return data_; }

  virtual bool encode(uint8_t *data) override
  {
    if (!isTxIntervalSatisfied()) {
      return false;
    }
    memcpy(data, &data_, sizeof(data_));
    last_encode_tick_ = getNowTickMs();
    return true;
  }

  void setUserData(const uint8_t *data, size_t length)
  {
    if (length > sizeof(data_.user_data)) {
      length = sizeof(data_.user_data);
    }
    memset(data_.user_data, 0, sizeof(data_.user_data));
    memcpy(data_.user_data, data, length);
  }

  bool setSenderId(RfrId id)
  {
    if (checkSenderId(id)) {
      data_.sender_id = id;
      return true;
    } else {
      return false;
    }
  }

  bool setReceiverId(RfrId id)
  {
    if (checkReceiverId(id)) {
      data_.receiver_id = id;
      return true;
    } else {
      return false;
    }
  }

  bool checkSenderId(RfrId id) const
  {
    if (ids::GetTeamColor(id) == team_color_ &&
        ids::GetIdType(id) == ids::IdType::kIdTypeRobot) {
      return true;
    }
    return false;
  }

  bool checkReceiverId(RfrId id) const
  {
    if (ids::GetTeamColor(id) == team_color_ &&
        ids::GetIdType(id) == ids::IdType::kIdTypeClient) {
      return true;
    }
    return false;
  }

 protected:
  Data data_ = {0};

  TeamColor team_color_ = TeamColor::kTeamColorError;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0308_INTER_CUSTOM_MSG_HPP_ */
