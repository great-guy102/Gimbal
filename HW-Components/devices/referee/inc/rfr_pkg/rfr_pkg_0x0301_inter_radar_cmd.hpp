/**
 * @file      rfr_pkg_0x0301_inter_radar_cmd.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0301_INTER_RADAR_CMD_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0301_INTER_RADAR_CMD_HPP_

/* Includes ------------------------------------------------------------------*/
#include "rfr_pkg_0x0301_inter_among_robots.hpp"

namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/** @struct InterRadarCmdData
 * @brief 雷达自主决策数据
 */
struct __REFEREE_PACKED InterRadarCmdData {
  /**
   * @brief 雷达是否确认触发双倍易伤的数值
   *
   * 开局为
   * 0，修改此值即可请求触发双倍易伤，若此时雷达拥有触发双倍易伤的机会，则可触发。
   *
   * @attention 此值的变化需要单调递增且每次仅能增加 1，否则视为不合法。
   * @attention 示例：此值开局仅能为 0，此后雷达可将其从 0 修改至
   * 1，若雷达拥有触发双倍易伤的机会，则触发双倍易伤。此后雷达可将其从 1 修改至
   * 2，以此类推。
   * @attention
   * 若雷达请求双倍易伤时，双倍易伤正在生效，则第二次双倍易伤将在第一次双倍易伤结束后生
   * 效。
   */
  uint8_t trigger_vulnerability_value;
};

class InterRadarCmd : public InterAmongRobotsPackage
{
 public:
  typedef InterRadarCmdData Data;

  virtual CmdId getInterCmdId(void) const override { return 0x0121; }
  virtual DataLength getInterDataLength(void) const override
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

  Data &getData(void) { return data_; }

 protected:
  virtual bool checkSenderId(RfrId id) const
  {
    return id == ids::RobotId::kRobotBlueRadar ||
           id == ids::RobotId::kRobotRedRadar;
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

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0301_INTER_RADAR_CMD_HPP_ */
