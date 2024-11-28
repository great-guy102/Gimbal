/**
 * @file      rfr_pkg_0x0104_team_warning.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0104_TEAM_WARNING_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0104_TEAM_WARNING_HPP_

/* Includes ------------------------------------------------------------------*/
#include "rfr_pkg_core.hpp"
namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/**
 * @enum PenaltyLevel
 * @brief 判罚等级枚举
 */
enum PenaltyLevel : uint8_t {
  kPenaltyLevelNone = 0u,   ///< 无判罚
  kPenaltyLevelBothYelow,   ///< 双方黄牌
  kPenaltyLevelYellowCard,  ///< 黄牌
  kPenaltyLevelRedCard,     ///< 红牌
  kPenaltyLevelForfeiture,  ///< 判负
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/**
 * @struct TeamRefereeWarningData
 * @brief 裁判警告数据
 */
struct __REFEREE_PACKED TeamRefereeWarningData {
  uint8_t level;     ///< 最后一次受到判罚的等级
  uint8_t robot_id;  ///< 最后一次受到判罚的违规机器人ID
  uint8_t count;     ///< 最后一次受到判罚的违规机器人对应判罚等级的违规次数
};
static_assert(sizeof(TeamRefereeWarningData) == 3, "TeamRefereeWarningData size error");

/** @class TeamRefereeWarningPackage
 * @brief 裁判警告数据包
 *
 * 数据说明：
 * - 命令码：0x0104
 * - 数据长度：3
 * - 发送频率：己方判罚/判负时触发发送，其余时间以 1Hz 频率
 * - 发送方/接收方：服务器->被判罚方全体机器人
 * - 常规链路
 */
class TeamRefereeWarningPackage : public ProtocolRxPackage
{
 public:
  typedef TeamRefereeWarningData Data;

  virtual CmdId getCmdId(void) const override { return 0x0104; }
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

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0104_TEAM_WARNING_HPP_ */
