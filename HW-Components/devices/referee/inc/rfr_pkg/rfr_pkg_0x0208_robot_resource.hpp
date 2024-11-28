/**
 * @file      rfr_pkg_0x0208_robot_resource.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0208_ROBOT_RESOURCE_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0208_ROBOT_RESOURCE_HPP_

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
 * @struct RobotResourceData
 * @brief 机器人剩余资源数据
 */
struct __REFEREE_PACKED RobotResourceData {
  uint16_t allowence_17mm;  ///< 17mm弹丸允许发弹量
  uint16_t allowence_42mm;  ///< 42mm弹丸允许发弹量
  uint16_t remaining_coin;  ///< 剩余金币数量
};
static_assert(sizeof(RobotResourceData) == 6, "RobotResourceData size error");
/** @class RobotResourcePackage
 * @brief 机器人剩余资源数据包
 *
 * 数据说明：
 * - 命令码：0x0208
 * - 数据长度：6
 * - 发送频率：10Hz
 * - 发送方/接收方：服务器->己方英雄、步兵、哨兵、空中机器人
 * - 所属数据链路：常规链路
 */
class RobotResourcePackage : public ProtocolRxPackage
{
 public:
  typedef RobotResourceData Data;

  virtual CmdId getCmdId(void) const override { return 0x0208; }
  virtual DataLength getDataLength(void) const override { return sizeof(Data); }
  virtual uint32_t getMaxRxIntervalMs(void) const override
  {
    return FREQ2INTERVAL(10);
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

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0208_ROBOT_RESOURCE_HPP_ */
