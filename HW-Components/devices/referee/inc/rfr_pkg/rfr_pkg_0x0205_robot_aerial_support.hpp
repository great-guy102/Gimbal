/**
 * @file      rfr_pkg_0x0205_robot_aerial_support.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0205_ROBOT_AERIAL_SUPPORT_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0205_ROBOT_AERIAL_SUPPORT_HPP_

/* Includes ------------------------------------------------------------------*/
#include "rfr_pkg_core.hpp"

namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/**
 * @enum AerialSupportStatus
 * @brief 空中支援状态枚举
 */
enum AerialSupportStatus {
  kAerialSupportStatusCooling = 0u,  ///< 空中机器人正在冷却
  kAerialSupportStatusReady = 1u,    ///< 空中支援冷却完毕
  kAerialSupportStatusOnging = 2u,   ///< 空中支援期间
};
/* Exported types ------------------------------------------------------------*/

/**
 * @struct RobotAerialSupportData
 * @brief 空中支援状态数据
 */
struct __REFEREE_PACKED RobotAerialSupportData {
  uint8_t AerialSupport_status;  ///< 空中机器人状态 @see AerialSupportStatus
  /**
   * 此状态的剩余时间，单位为秒，向下取整，即冷却时间剩余1.9秒时，此值为1。
   * 若冷却时间为0，但未呼叫空中支援，则该值为0
   */
  uint8_t time_remain;
};
static_assert(sizeof(RobotAerialSupportData) == 2, "RobotAerialSupportData size error");
// ! 数据结构体大小与汇总表中数据段长度不一致，等待实际检测

/** @class RobotAerialSupportPackage
 * @brief 空中支援状态数据包
 *
 * 数据说明：
 * - 命令码：0x0205
 * - 数据长度：2
 * - 发送频率：1Hz
 * - 发送方/接收方：服务器->己方空中机器人
 * - 所属数据链路：常规链路
 *
 * @attention 在《串口协议v1.5》 P6 表 2-1 中，0x0205 的 数据段长度 字段为 1 ，与
 *            P14 表 2-12 中的 大小 字段和结构体的定义不一致，此处以结构体大小为准。
 */
class RobotAerialSupportPackage : public ProtocolRxPackage
{
 public:
  typedef RobotAerialSupportData Data;

  virtual CmdId getCmdId(void) const override { return 0x0205; }
  virtual DataLength getDataLength(void) const override { return sizeof(Data); }
  virtual uint32_t getMaxRxIntervalMs(void) const override {
    return FREQ2INTERVAL(10); }

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
};  // namespace referee
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0205_ROBOT_AERIAL_SUPPORT_HPP_ */
