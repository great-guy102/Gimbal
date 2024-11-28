/**
 * @file      rfr_pkg_0x0105_team_dart_status.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0105_TEAM_DART_STATUS_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0105_TEAM_DART_STATUS_HPP_

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
 * @struct TeamDartStatusData
 * @brief 飞镖机器人状态数据
 */
struct __REFEREE_PACKED TeamDartStatusData {
  uint8_t dart_remaining_time;       ///< 飞镖发射剩余时间，单位：秒
  /**
   * 最近一次己方飞镖击中的目标，开局默认为 0，1 为击中前哨站，2 为击中基地固定目标，
   * 3 为击中基地随机目标
   */
  uint16_t last_hit_target : 2;
  /** 对方最近被击中的目标累计被击中计数，开局默认为 0，至多为 4 */
  uint16_t recent_hit_count : 3;
  /**
   * 飞镖此时选定的击打目标，开局默认或未选定/选定前哨站时为 0，选中基地固定目标为 1，
   * 选中基地随机目标为 2 */
  uint16_t selected_hit_target : 2;
  uint16_t reserved : 9;             ///< 保留位
};
static_assert(sizeof(TeamDartStatusData) == 3, "TeamDartStatusData size error");
/** @class TeamDartStatusPackage
 * @brief 飞镖机器人状态数据包
 *
 * 数据说明：
 * - 命令码：0x0105
 * - 数据长度：3
 * - 发送频率：1Hz
 * - 发送方/接收方：服务器->己方全体机器人
 * - 所属数据链路：常规链路
 */
class TeamDartStatusPackage : public ProtocolRxPackage
{
 public:
  typedef TeamDartStatusData Data;

  virtual CmdId getCmdId(void) const override { return 0x0105; }
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

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0105_TEAM_DART_STATUS_HPP_ */
