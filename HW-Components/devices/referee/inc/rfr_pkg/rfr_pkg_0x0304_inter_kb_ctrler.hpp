/**
 * @file      rfr_pkg_0x0304_inter_kb_ctrler.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0304_INTER_KB_CTRLER_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0304_INTER_KB_CTRLER_HPP_

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
 * @struct InterKbCtrlerMouse
 * @brief 鼠标数据
 */
struct __REFEREE_PACKED InterKbCtrlerMouse {
  int16_t x_vel;      ///< 鼠标 x 轴移动速度，负值标识向左移动
  int16_t y_vel;      ///< 鼠标 y 轴移动速度，负值标识向下移动
  int16_t wheel_vel;  ///< 鼠标滚轮滚动速度，负值标识向后滚动
  uint8_t left;       ///< 鼠标左键是否按下：0 为没按下；1 为按下
  uint8_t right;      ///< 鼠标右键是否按下：0 为没按下；1 为按下
};
/**
 * @struct InterKbCtrlerKeyboard
 * @brief 键盘遥控数据
 */
struct __REFEREE_PACKED InterKbCtrlerKeyboard {
  uint16_t w : 1;      ///< 键盘 w 键是否按下：0 为没按下；1 为按下
  uint16_t s : 1;      ///< 键盘 s 键是否按下：0 为没按下；1 为按下
  uint16_t a : 1;      ///< 键盘 a 键是否按下：0 为没按下；1 为按下
  uint16_t d : 1;      ///< 键盘 d 键是否按下：0 为没按下；1 为按下
  uint16_t shift : 1;  ///< 键盘 shift 键是否按下：0 为没按下；1 为按下
  uint16_t ctrl : 1;   ///< 键盘 ctrl 键是否按下：0 为没按下；1 为按下
  uint16_t q : 1;      ///< 键盘 q 键是否按下：0 为没按下；1 为按下
  uint16_t e : 1;      ///< 键盘 e 键是否按下：0 为没按下；1 为按下
  uint16_t r : 1;      ///< 键盘 r 键是否按下：0 为没按下；1 为按下
  uint16_t f : 1;      ///< 键盘 f 键是否按下：0 为没按下；1 为按下
  uint16_t g : 1;      ///< 键盘 g 键是否按下：0 为没按下；1 为按下
  uint16_t z : 1;      ///< 键盘 z 键是否按下：0 为没按下；1 为按下
  uint16_t x : 1;      ///< 键盘 x 键是否按下：0 为没按下；1 为按下
  uint16_t c : 1;      ///< 键盘 c 键是否按下：0 为没按下；1 为按下
  uint16_t v : 1;      ///< 键盘 v 键是否按下：0 为没按下；1 为按下
  uint16_t b : 1;      ///< 键盘 b 键是否按下：0 为没按下；1 为按下
};

/**
 * @struct InterKbCtrlerData
 * @brief 键鼠遥控数据
 */
struct __REFEREE_PACKED InterKbCtrlerData {
  InterKbCtrlerMouse mouse;  ///< 鼠标数据
  union {
    InterKbCtrlerKeyboard key;  ///< 键盘数据
    uint16_t raw;               ///< 原始数据，调试用
  } kb;                         ///< 键盘数据
  uint16_t reserved;            ///< 保留字段
};
static_assert(sizeof(InterKbCtrlerData) == 12, "InterKbCtrlerData size error");
/**
 * @class InterMapClientToRobotPackage
 * @brief 键鼠遥控数据包
 *
 * 数据说明：
 *
 * - 命令码：0x0304
 * - 数据长度：12
 * - 发送频率：30Hz
 * - 发送方/接收方：客户端→选手端图传连接的机器人
 * - 所属数据链路：图传链路
 */
class InterKbCtrlerPackage : public ProtocolRxPackage
{
 public:
  typedef InterKbCtrlerData Data;

  virtual CmdId getCmdId(void) const override { return 0x0304; }
  virtual DataLength getDataLength(void) const override { return sizeof(Data); }
  virtual uint32_t getMaxRxIntervalMs(void) const override
  {
    return FREQ2INTERVAL(30);
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

 protected:
  Data data_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0304_INTER_KB_CTRLER_HPP_ */
