/**
 * @file      rfr_pkg_0x0203_robot_pos.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0203_ROBOT_POS_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0203_ROBOT_POS_HPP_

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
 * @struct RobotPosData
 * @brief 机器人位置数据
 */
struct __REFEREE_PACKED RobotPosData {
  float x;      ///< 本机器人位置x坐标，单位：m
  float y;      ///< 本机器人位置y坐标，单位：m
  float angle;  ///< 本机器人测速模块的朝向，单位：度。正北为0度
};
static_assert(sizeof(RobotPosData) == 12, "RobotPosData size error");
/** @class RobotPosPackage
 * @brief 机器人位置数据包
 *
 * 数据说明：
 * - 命令码：0x0203
 * - 数据长度：12
 * - 发送频率：1Hz
 * - 发送方/接收方：服务器->对应机器人
 * @attention 在《串口协议v1.6.1》 P6 表 2-1 中，0x0203 的 数据段长度 字段为 16 ，与
 *            P14 表 2-11 中的 大小 字段和结构体的定义不一致，此处以结构体大小为准。
 * TODO: 后续验证数据段长度是否为 12
 */
class RobotPosPackage : public ProtocolRxPackage
{
 public:
  typedef RobotPosData Data;

  virtual CmdId getCmdId(void) const override { return 0x0203; }
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

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0203_ROBOT_POS_HPP_ */
