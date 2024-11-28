/**
 * @file      rfr_pkg_0x020E_robot_radar_decision.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X020E_ROBOT_RADAR_DECISION_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X020E_ROBOT_RADAR_DECISION_HPP_

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
 * @struct RobotRadarDecisionData
 * @brief 储存雷达机器人的双倍易伤信息
 */
struct __REFEREE_PACKED RobotRadarDecisionData {
  /*/< 雷达是否拥有触发双倍易伤的机会，开局为 0，数值为雷达拥有触发双倍易伤的机会，
       至多为 2 */
  uint8_t double_vuln_chances : 2;
  uint8_t double_vuln : 1;          ///< 对方是否正在被触发双倍易伤
  uint8_t reserved : 5;             ///< 保留位
};
static_assert(sizeof(RobotRadarDecisionData) == 1, "RobotRadarDecisionData size error");
/** @class RobotRadarDecisionPackage
 * @brief 雷达自主决策信息同步
 *
 * 数据说明：
 * - 命令码：0x020E
 * - 数据长度：1
 * - 发送频率：1Hz
 * - 发送方/接收方：服务器->雷达机器人
 * - 所属数据链路：常规链路
 */
class RobotRadarDecisionPackage : public ProtocolRxPackage
{
 public:
  typedef RobotRadarDecisionData Data;

  virtual CmdId getCmdId(void) const override { return 0x020E; }
  virtual DataLength getDataLength(void) const override { return sizeof(Data); }
  virtual uint32_t getMaxRxIntervalMs(void) const override
  {
    return FREQ2INTERVAL(1);
  }

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

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X020E_ROBOT_RADAR_DECISION_HPP_ */
