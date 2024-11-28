/**
 * @file      rfr_pkg_0x020D_robot_sentry_decision.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X020D_ROBOT_SENTRY_DECISION_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X020D_ROBOT_SENTRY_DECISION_HPP_

/* Includes ------------------------------------------------------------------*/
#include "rfr_pkg_core.hpp"

namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/**
 * @struct RobotSentryDecisionData
 * @brief 储存哨兵机器人的兑换信息
 */
struct __REFEREE_PACKED RobotSentryDecisionData {
  uint32_t allowance : 11;        ///< 除远程兑换外，哨兵成功兑换的发弹量
  uint32_t remote_allowance : 4;  ///< 哨兵成功远程兑换发弹量的次数
  uint32_t remote_hp : 4;         ///< 哨兵成功远程兑换血量的次数
  uint32_t allow_free_resurrection : 1;        ///< 哨兵当前是否可以确认免费复活
  uint32_t allow_redemption_resurrection : 1;  ///< 哨兵当前是否可以兑换立即复活
  uint32_t redemption_resurrection_cost : 10;  ///< 哨兵当前兑换立即复活需要花费的金币数
  uint32_t reserved1 : 1;
  uint16_t out_of_war_status : 1;        ///< 哨兵当前是否处于脱战状态
  uint16_t remain_total_allowance : 11;  ///< 队伍17mm允许发弹量的剩余可兑换数
  uint16_t reserved2 : 4;
};
static_assert(sizeof(RobotSentryDecisionData) == 6, "RobotSentryDecisionData size error");
/** @class RobotSentryDecisionPackage
 * @brief 哨兵自主决策信息同步
 *
 * 数据说明：
 * - 命令码：0x020D
 * - 数据长度：6
 * - 发送频率：1Hz
 * - 发送方/接收方：服务器->哨兵机器人
 * - 所属数据链路：常规链路
 * @see RobotSentryDecisionData
 */
class RobotSentryDecisionPackage : public ProtocolRxPackage
{
 public:
  typedef RobotSentryDecisionData Data;

  virtual CmdId getCmdId(void) const override { return 0x020D; }
  virtual DataLength getDataLength(void) const override { return sizeof(Data); }
  virtual uint32_t getMaxRxIntervalMs(void) const override {
    return FREQ2INTERVAL(1); }

  const Data &getData(void) const { return data_; }

  virtual bool decode(const CmdId &cmd_id, const uint8_t *data_ptr) override
  {
    if (cmd_id == getCmdId()) {
      memcpy(&data_, data_ptr, sizeof(Data));
      last_decode_tick_ = getNowTickMs();
      is_handled_ = false;
      return true;
    }
    return false;
  }

 private:
   Data data_ = {0};
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X020D_ROBOT_SENTRY_DECISION_HPP_ */
