/**
 * @file      rfr_pkg_0x0302_inter_custom_ctrler.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-02-18
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
 * | 1.0.0 | 2024-MM-DD | ZhouShichan | 首次完成 |
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0302_INTER_CUSTOM_CTRLER_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0302_INTER_CUSTOM_CTRLER_HPP_

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
 * @struct InterCustomCtrlerData
 * @brief 自定义控制器与机器人交互数据
 */
struct __REFEREE_PACKED InterCustomCtrlerData {
  uint8_t data[30];  ///< 自定义数据
};
static_assert(sizeof(InterCustomCtrlerData) == 30, "InterCustomCtrlerData size error");
/**
 * @class InterCustomCtrlerPackage
 * @brief 自定义控制器与机器人交互数据包
 *
 * 操作手可使用自定义控制器通过图传链路向对应的机器人发送数据。
 *
 * 数据说明：
 *
 * - 命令码：0x0302
 * - 数据长度：30
 * - 发送频率：发送方触发发送, 频率上限为 30Hz
 * - 发送方/接收方：自定义控制器->选手端图传连接的机器人
 * - 所属数据链路：图传链路
 */
class InterCustomCtrlerPackage : public ProtocolRxPackage
{
 public:
  typedef InterCustomCtrlerData Data;

  virtual CmdId getCmdId(void) const override { return 0x0302; }
  virtual DataLength getDataLength(void) const override { return sizeof(Data); }
  virtual uint32_t getMaxRxIntervalMs(void) const override { 
    return FREQ2INTERVAL(30); }

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

 protected:
  Data data_ = {0};
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0302_INTER_CUSTOM_CTRLER_HPP_ */
