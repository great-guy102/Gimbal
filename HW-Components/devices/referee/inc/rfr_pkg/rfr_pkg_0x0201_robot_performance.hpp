/**
 * @file      rfr_pkg_0x0201_robot_performance.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0201_TEAM_WARNING_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0201_TEAM_WARNING_HPP_

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
 * @struct RobotPerformanceData
 * @brief 机器人性能体系数据
 */
struct __REFEREE_PACKED RobotPerformanceData {
  uint8_t robot_id;                             ///< 本机器人ID
  uint8_t robot_level;                          ///< 机器人等级
  uint16_t current_hp;                          ///< 机器人当前血量
  uint16_t maximum_hp;                          ///< 机器人血量上限
  uint16_t shooter_barrel_cooling_value;        ///< 机器人枪口热量每秒冷却值
  uint16_t shooter_barrel_heat_limit;           ///< 机器人枪口热量上限
  uint16_t chassis_power_limit;                 ///< 底盘功率限制
  /*/< 电源管理模块 gimbal 口输出：0-关闭，1-开启 */
  uint8_t power_management_gimbal_output : 1;
  /*/< 电源管理模块 chassis 口输出：0-关闭，1-开启 */
  uint8_t power_management_chassis_output : 1;
  /*/< 电源管理模块 shooter 口输出：0-关闭，1-开启 */
  uint8_t power_management_shooter_output : 1;
  uint8_t reserved : 5;                         ///< 保留
};
static_assert(sizeof(RobotPerformanceData) == 13, "RobotPerformanceData size error");
/** @class RobotPerformancePackage
 * @brief 机器人性能体系数据包
 *
 * 数据说明：
 * - 命令码：0x0201
 * - 数据长度：13
 * - 发送频率：10Hz
 * - 发送方/接收方：主控模块->对应机器人
 * - 所属数据链路：常规链路
 * @attention 经实际测试，该数据包需要连接服务器才会收到对应数据。在离线调试时，需要手
 *            动模拟数据。
 */
class RobotPerformancePackage : public ProtocolRxPackage
{
 public:
  typedef RobotPerformanceData Data;

  virtual CmdId getCmdId(void) const override { return 0x0201; }
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

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0201_TEAM_WARNING_HPP_ */
