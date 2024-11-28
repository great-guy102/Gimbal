/**
 * @file      rfr_pkg_0x0301_inter_sentry_cmd.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0301_INTER_SENTRY_CMD_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0301_INTER_SENTRY_CMD_HPP_

/* Includes ------------------------------------------------------------------*/
#include "rfr_pkg_0x0301_inter_among_robots.hpp"

namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/** @struct InterSentryCmdData
 * @brief 哨兵自主决策数据
 *
 * 在哨兵发送该子命令时，服务器将按照从相对低位到相对高位的原则依次处理这些指令，直至全
 * 部成功或不能处理为止。
 *
 * 示例：若队伍金币数为 0，此时哨兵战亡，“是否确认复活”的值为 1，“是否确认兑换立即复活”
 * 的值为 1，“确认兑换的允许发弹量值”为 100。（假定之前哨兵未兑换过允许发弹量）由于此时
 * 队伍金币数不足以使哨兵兑换立即复活，则服务器将会忽视后续指令，等待哨兵发送的下一组指令
 */
struct __REFEREE_PACKED InterSentryCmdData {
  /**
   * @brief 哨兵机器人是否确认复活
   *
   * - 0 表示哨兵机器人确认不复活，即使此时哨兵的复活读条已经完成
   * - 1 表示哨兵机器人确认复活，若复活读条完成将立即复活
   */
  uint32_t is_to_revive : 1;
  /**
   * @brief 哨兵机器人是否确认兑换立即复活
   *
   * - 0 表示哨兵机器人确认不兑换立即复活
   * - 1
   * 表示哨兵机器人确认兑换立即复活，若此时哨兵机器人符合兑换立即复活的规则要求，则会立
   * 即消耗金币兑换立即复活
   */
  uint32_t is_to_instant_revive : 1;
  /**
   * @brief 哨兵将要兑换的发弹量值，开局为
   * 0，修改此值后，哨兵在补血点即可兑换允许发弹量。
   *
   * @attention 此值的变化需要单调递增，否则视为不合法。
   * @attention 示例：此值开局仅能为 0，此后哨兵可将其从 0 修改至 X，则消耗 X 金币成功
   * 兑换 X 允许发弹量。此后哨兵可将其从 X 修改至X+Y，以此类推。
   */
  uint32_t num_exchanging_projectiles : 11;
  /**
   * @brief 哨兵远程兑换发弹量的请求次数，开局为
   * 0，修改此值即可请求远程兑换发弹量。
   *
   * @attention 此值的变化需要单调递增且每次仅能增加 1，否则视为不合法。
   * @attention 示例：此值开局仅能为 0，此后哨兵可将其从 0 修改至 1，则消耗金币远程兑
   * 换允许发弹量。此后哨兵可将其从 1 修改至 2，以此类推。
   */
  uint32_t num_requests_remote_proj : 4;
  /**
   * @brief 哨兵远程兑换血量的请求次数，开局为 0，修改此值即可请求远程兑换血量。
   *
   * @attention 此值的变化需要单调递增且每次仅能增加 1，否则视为不合法。
   * @attention 示例：此值开局仅能为 0，此后哨兵可将其从 0 修改至 1，则消耗金币远程兑
   * 换血量。此后哨兵可将其从 1 修改至 2，以此类推。
   */
  uint32_t num_requests_remote_hp : 4;
  uint32_t resrved : 11;  ///< 保留位
};

/**
 * @brief 哨兵自主决策数据包
 *
 */
class InterSentryCmd : public InterAmongRobotsPackage
{
 public:
  typedef InterSentryCmdData Data;

  virtual CmdId getInterCmdId() const override { return 0x0120; }
  virtual DataLength getInterDataLength() const override
  {
    return sizeof(Data);
  }

  virtual bool setSenderId(RfrId id) override
  {
    if (checkSenderId(id)) {
      sender_id_ = id;
      receiver_id_ = ids::ServerId::kServerId;
      return true;
    }
    return false;
  }

  Data &getData() { return data_; }

 protected:
  virtual bool checkSenderId(RfrId id) const
  {
    return id == ids::RobotId::kRobotBlueSentry ||
           id == ids::RobotId::kRobotRedSentry;
  }

  virtual bool checkReceiverId(RfrId id) const
  {
    return id == ids::ServerId::kServerId;
  }

  virtual void encodeInterData(uint8_t *data) override
  {
    memcpy(data, &data_, sizeof(Data));
  }

  Data data_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0301_INTER_SENTRY_CMD_HPP_ */
