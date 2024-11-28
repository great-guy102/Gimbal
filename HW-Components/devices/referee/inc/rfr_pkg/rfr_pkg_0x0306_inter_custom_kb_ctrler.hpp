/**
 * @file      rfr_pkg_0x0306_inter_custom_kb_ctrler.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0306_INTER_CUSTOM_KB_CTRLER_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0306_INTER_CUSTOM_KB_CTRLER_HPP_

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
 * @struct InterCustomKbCtrlerData
 * @brief 自定义键鼠控制器与机器人交互数据
 */
struct __REFEREE_PACKED InterCustomKbCtrlerData {
  /**
   * 键盘键值：
   *
   * - bit 0-7：按键 1 键值
   * - bit 8-15：按键 2 键值
   *
   * @note - 仅响应选手端开放的按键
   * @note - 使用通用键值，支持 2 键无冲，键值顺序变更不会改变按下状态，若无新的按键信
   *         息，将保持上一帧数据的按下状态
   */
  uint16_t key_value;
  /**
   * 鼠标 X 轴像素位置
   *
   * @note 位置信息使用绝对像素点值（赛事客户端使用的分辨率为 1920×1080，屏幕左上角为
   *       （0，0））
   */
  uint16_t x_position : 12;
  /**
   * 鼠标左键状态
   *
   * @note 鼠标按键状态 1 为按下，其他值为未按下，仅在出现鼠标图标后响应该信息，若无新
   *       的鼠标信息，选手端将保持上一帧数据的鼠标信息，当鼠标图标消失后该数据不再保持
   */
  uint16_t mouse_left : 4;
  /**
   * 鼠标 Y 轴像素位置
   *
   * @note 位置信息使用绝对像素点值（赛事客户端使用的分辨率为 1920×1080，屏幕左上角为
   *       （0，0））
   */
  uint16_t y_position : 12;
  /**
   * 鼠标右键状态
   *
   * @note 鼠标按键状态 1 为按下，其他值为未按下，仅在出现鼠标图标后响应该信息，若无新
   *       的鼠标信息，选手端将保持上一帧数据的鼠标信息，当鼠标图标消失后该数据不再保持
   */
  uint16_t mouse_right : 4;
  /** 保留位 */
  uint16_t reserved;
};
static_assert(sizeof(InterCustomKbCtrlerData) == 8, "InterCustomKbCtrlerData size error");
/**
 * @class InterCustomKbCtrler
 * @brief 自定义控制器与选手端交互数据
 *
 * 数据说明：
 *
 * - 命令码：0x0306
 * - 数据长度：8
 * - 发送频率：发送方触发发送，频率上限为 30Hz
 * - 发送方/接收方：自定义控制器->选手端
 * - 所属数据链路：非链路数据
 */
class InterCustomKbCtrler : public ProtocolTxPackage
{
 public:
  typedef InterCustomKbCtrlerData Data;

  virtual CmdId getCmdId(void) const override { return 0x0306; }
  virtual DataLength getDataLength(void) const override { return sizeof(Data); }
  virtual uint32_t getMinTxIntervalMs() const override {
    return FREQ2INTERVAL(30); }

  const Data &getData(void) const { return data_; }

  virtual bool encode(uint8_t *data) override
  {
    if (!isTxIntervalSatisfied()) {
      return false;
    }
    memcpy(data, &data_, sizeof(Data));
    last_encode_tick_ = getNowTickMs();
    return true;
  }

 protected:
  Data data_ = {0};
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0306_INTER_CUSTOM_KB_CTRLER_HPP_ */
