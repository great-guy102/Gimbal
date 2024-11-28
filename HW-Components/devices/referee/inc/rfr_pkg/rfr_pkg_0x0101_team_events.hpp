/**
 * @file      rfr_pkg_0x0101_team_events.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0101_TEAM_EVENTS_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0101_TEAM_EVENTS_HPP_

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
 * @struct TeamEventData
 * @brief 场地事件数据
 */
struct __REFEREE_PACKED TeamEventData {
  uint32_t restoration_1 : 1;  ///< 己方补给站前补血点的占领状态，1 为已占领
  uint32_t restoration_2 : 1;  ///< 己方补给站内部补血点的占领状态，1 为已占领
  /** 己方补给站的占领状态，1 为已占领 @attention 仅 RMUL 适用 */
  uint32_t supplier_area : 1;
  /** 己方能量机关激活点的占领状态，1 为已占领 */
  uint32_t power_rune_activation_point : 1;
  uint32_t small_power_rune : 1;  ///< 己方小能量机关的激活状态，1 为已激活
  uint32_t large_power_rune : 1;  ///< 己方大能量机关的激活状态，1 为已激活
  /** 己方环形高地的占领状态，1 为被己方占领，2 为被对方占领 */
  uint32_t ring_elevated_ground : 2;
  /** 己方（3号）梯形高地的占领状态，1 为被己方占领，2 为被对方占领 */
  uint32_t trapezoid_elevated_ground_r3b3 : 2;
  /** 己方（4号）梯形高地的占领状态，1 为被己方占领，2 为被对方占领 */
  uint32_t trapezoid_elevated_ground_r4b4 : 2;
  /** 己方基地虚拟护盾的剩余值百分比（四舍五入，保留整数） */
  uint32_t base_virtual_shield : 7;
  /** 飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为0） */
  uint32_t dart_last_hit_us_time : 9;
  /**
   * 飞镖最后一次击中己方前哨站或基地的具体目标
   * 开局默认为 0, 1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机目标
   */
  uint32_t dart_last_hit_us_type : 2;
  /**
   * 中心增益点的占领情况
   * 0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领。
   * @attention 仅 RMUL 适用
   */
  uint32_t center_buff_area : 2;
};
static_assert(sizeof(TeamEventData) == 4, "TeamEventData size error");

/** @class TeamEventPackage
 * @brief 场地事件数据包
 *
 * 数据说明：
 * - 命令码：0x0101
 * - 数据长度：4
 * - 发送频率：1Hz
 * - 发送方/接收方：服务器->己方全体机器人
 * - 所属数据链路：常规链路
 */
class TeamEventPackage : public ProtocolRxPackage
{
 public:
  typedef TeamEventData Data;

  virtual CmdId getCmdId(void) const override { return 0x0101; }
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

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0101_TEAM_EVENTS_HPP_ */
