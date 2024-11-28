/**
 * @file      rfr_pkg_0x0301_inter_among_robots.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-25
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
 * | 1.0.0 | 2024-02-18 | ZhouShichan | 首次完成 |
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0301_INTER_AMONG_ROBOTS_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0301_INTER_AMONG_ROBOTS_HPP_

/* Includes ------------------------------------------------------------------*/
#include "rfr_id.hpp"
#include "rfr_pkg_core.hpp"

namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
const DataLength kMaxDataLengthOf0x0301 = 112;  ///< 0x0301 数据包的最大数据长度
/* Exported types ------------------------------------------------------------*/

struct __REFEREE_PACKED InterConfig {
  CmdId data_cmd_id;
  uint16_t sender_id;
  uint16_t receiver_id;
};

/**
 * @class InterAmongRobotsPackage
 * @brief 机器人间交互数据包
 *
 * 机器人间交互数据包的基类，用于定义机器人间交互数据包的基本结构。
 *
 * 数据说明：
 * - 命令码：0x0301
 * - 数据长度：最大 112
 * - 发送频率：发送方触发发送，最大 30 Hz
 * - 发送方/接收方：机器人->机器人 或 机器人->服务器
 * - 所属数据链路：常规链路
 */
class InterAmongRobotsPackage : public ProtocolTxPackage
{
 public:
  virtual CmdId getCmdId(void) const override { return 0x0301; }
  virtual DataLength getDataLength(void) const override
  {
    return sizeof(InterConfig) + getInterDataLength();
  }
  virtual uint32_t getMinTxIntervalMs() const override
  {
    return FREQ2INTERVAL(30);
  }

  virtual CmdId getInterCmdId() const = 0;
  virtual DataLength getInterDataLength() const = 0;

  virtual bool setSenderId(RfrId id)
  {
    if (checkSenderId(id)) {
      sender_id_ = id;
      return true;
    } else {
      return false;
    }
  }

  virtual bool setReceiverId(RfrId id)
  {
    if (checkReceiverId(id)) {
      receiver_id_ = id;
      return true;
    } else {
      return false;
    }
  }

  virtual bool encode(uint8_t *data) override;

  virtual bool isTxIntervalSatisfied() const override;

 protected:
  virtual bool checkSenderId(RfrId id) const { return false; }

  virtual bool checkReceiverId(RfrId id) const { return false; }

  /**
   * @brief       编码交互数据包的子内容，需要在子类中实现。外部则调用此类的 encode 函
   *              数
   * @param        data: 待编码的数据包
   * @retval       None
   * @note        None
   */
  virtual void encodeInterData(uint8_t *data) = 0;

  RfrId sender_id_ = 0x0000;    ///< 发送方 ID
  RfrId receiver_id_ = 0x0000;  ///< 接收方 ID

 private:
  /*/< 上一次发送 0x0301 数据包的时间戳，单位 ms，父类与子类共享 */
  static uint32_t last_encode_tick_of_0x0301_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0301_INTER_AMONG_ROBOTS_HPP_ */
