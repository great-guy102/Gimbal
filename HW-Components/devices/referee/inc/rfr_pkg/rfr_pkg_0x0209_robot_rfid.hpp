/**
 * @file      rfr_pkg_0x0209_robot_rfid.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0209_ROBOT_RFID_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0209_ROBOT_RFID_HPP_

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
 * @struct __REFEREE_PACKED RegionsWithRFID
 * @brief 机器人 RFID 模块状态数据
 *
 * 这个结构体通过位字段存储了RFID探测到的每一个增益点的信息。
 * bit 位值为 1/0 的含义：是否已检测到该增益点 RFID 卡
 * @note 只有在比赛内探测到的增益点才会生效，否则，即使检测到对应的RFID卡，对应值也为0.
 */
struct __REFEREE_PACKED RobotRfidData {
  uint32_t our_base : 1;                 ///< 己方基地增益点
  uint32_t our_ring_elevated : 1;        ///< 己方环形高地增益点
  uint32_t opp_ring_elevated : 1;        ///< 对方环形高地增益点
  uint32_t our_trap_r3b3_elevated : 1;   ///< 己方 R3/B3 梯形高地增益点
  uint32_t opp_trap_r3b3_elevated : 1;   ///< 对方 R3/B3 梯形高地增益点
  uint32_t our_trap_r4b4_elevated : 1;   ///< 己方 R4/B4 梯形高地增益点
  uint32_t opp_trap_r4b4_elevated : 1;   ///< 对方 R4/B4 梯形高地增益点
  uint32_t our_energy_activation : 1;    ///< 己方能量机关激活点
  uint32_t our_launch_front : 1;         ///< 己方飞坡增益点（靠近己方一侧飞坡前）
  uint32_t our_launch_back : 1;          ///< 己方飞坡增益点（靠近己方一侧飞坡后）
  uint32_t opp_launch_front : 1;         ///< 对方飞坡增益点（靠近对方一侧飞坡前）
  uint32_t opp_launch_back : 1;          ///< 对方飞坡增益点（靠近对方一侧飞坡后）
  uint32_t our_outpost : 1;              ///< 己方前哨站增益点
  uint32_t our_healing : 1;              ///< 己方补血点（检测到任一均视为激活）
  uint32_t our_sentinel : 1;             ///< 己方哨兵巡逻区
  uint32_t opp_sentinel : 1;             ///< 对方哨兵巡逻区
  uint32_t our_big_resource_island : 1;  ///< 己方大资源岛增益点
  uint32_t opp_big_resource_island : 1;  ///< 对方大资源岛增益点
  uint32_t our_exchange_area : 1;        ///< 己方兑换区
  uint32_t central_boost : 1;            ///< 中心增益点 @attention 仅RMUL适用
  uint32_t reserved : 12;                ///< 保留区域
};
static_assert(sizeof(RobotRfidData) == 4, "RobotRfidData size error");
/** @class RobotRfidPackage
 * @brief 机器人 RFID 模块状态数据包
 *
 * 数据说明：
 * - 命令码：0x0209
 * - 数据长度：4
 * - 发送频率：3Hz
 * - 发送方/接收方：服务器->己方装有RFID模块的机器人
 * - 所属数据链路：常规链路
 */
class RobotRfidPackage : public ProtocolRxPackage
{
 public:
  typedef RobotRfidData Data;

  virtual CmdId getCmdId(void) const override { return 0x0209; }
  virtual DataLength getDataLength(void) const override { return sizeof(Data); }
  virtual uint32_t getMaxRxIntervalMs(void) const override
  {
    return FREQ2INTERVAL(3);
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

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0209_ROBOT_RFID_HPP_ */
