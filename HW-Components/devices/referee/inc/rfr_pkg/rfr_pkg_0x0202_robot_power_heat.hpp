/**
 * @file      rfr_pkg_0x0202_robot_power_heat.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0202_ROBOT_POWER_HEAT_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0202_ROBOT_POWER_HEAT_HPP_

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
 * @struct RobotPowerHeatData
 * @brief 实时底盘功率和枪口热量数据
 */
struct __REFEREE_PACKED RobotPowerHeatData {
  uint16_t chassis_voltage;  ///< 电源管理模块的chassis口输出电压，单位：mV
  uint16_t chassis_current;  ///< 电源管理模块的chassis口输出电流，单位：mA

  float chassis_power;                  ///< 底盘功率，单位：W
  uint16_t buffer_energy;               ///< 缓冲能量，单位：J
  uint16_t shooter_17mm_1_barrel_heat;  ///< 第一个17mm发射机构的枪口热量
  uint16_t shooter_17mm_2_barrel_heat;  ///< 第二个17mm发射机构的枪口热量
  uint16_t shooter_42mm_barrel_heat;    ///< 42mm发射机构的枪口热量
};
static_assert(sizeof(RobotPowerHeatData) == 16, "RobotPowerHeatData size error");
/** @class RobotPowerHeatPackage
 * @brief 实时底盘功率和枪口热量数据包
 *
 * 数据说明：
 * - 命令码：0x0202
 * - 数据长度：16
 * - 发送频率：50Hz
 * - 发送方/接收方：服务器->对应机器人
 * - 所属数据链路：常规链路
 */
class RobotPowerHeatPackage : public ProtocolRxPackage
{
 public:
  typedef RobotPowerHeatData Data;

  virtual CmdId getCmdId(void) const override { return 0x0202; }
  virtual DataLength getDataLength(void) const override { return sizeof(Data); }
  virtual uint32_t getMaxRxIntervalMs(void) const override
  {
    return FREQ2INTERVAL(50);
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

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0202_ROBOT_POWER_HEAT_HPP_ */
