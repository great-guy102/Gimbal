/**
 * @file      rfr_pkg_0x020A_robot_dart_client_cmd.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X020A_ROBOT_DART_CLIENT_CMD_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X020A_ROBOT_DART_CLIENT_CMD_HPP_

/* Includes ------------------------------------------------------------------*/
#include "rfr_pkg_core.hpp"

namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/**
 * @enum DartStationStatus
 * @brief 飞镖发射站的状态枚举
 */
enum DartStationStatus {
  kDartStationStatusOpened = 0u,  ///< 飞镖发射站已经开启
  kDartStationStatusClosed = 1u,  ///< 飞镖发射站关闭
  kDartStationStatusMoving = 2u   ///< 飞镖发射站正在开启或正在关闭
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @struct RobotDartClientCmdData
 * @brief 飞镖客户端的命令数据
 */
struct __REFEREE_PACKED RobotDartClientCmdData {
  /*/< 当前飞镖发射站的状态。 @see DartStationStatus */
  uint8_t dart_launch_opening_status;
  uint8_t reserved;                    ///< 保留
  /*/< 切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为 0 */
  uint16_t target_change_time;
  /*/< 最后一次操作手确定发射指令时的比赛剩余时间，单位：秒，初始值为 0 */
  uint16_t latest_launch_cmd_time;
};
static_assert(sizeof(RobotDartClientCmdData) == 6, "RobotDartClientCmdData size error");
/** @class RobotDartClientCmdPackage
 * @brief 飞镖客户端命令数据包
 *
 * 数据说明：
 * - 命令码：0x020A
 * - 数据长度：6
 * - 发送频率：3Hz
 * - 发送方/接收方：服务器->飞镖机器人
 * - 所属数据链路：常规链路
 */
class RobotDartClientCmdPackage : public ProtocolRxPackage
{
 public:
  typedef RobotDartClientCmdData Data;

  virtual CmdId getCmdId(void) const override { return 0x020A; }
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

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X020A_ROBOT_DART_CLIENT_CMD_HPP_ */
