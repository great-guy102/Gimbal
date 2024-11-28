/**
 *******************************************************************************
 * @file      : ET08A.hpp
 * @brief     : WFLY 天地飞 ET08A 遥控器
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *  继承于 Sbus 类，注意事项请看 sbus.hpp 文件
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_REMOTE_CONTROL_ET08A_HPP_
#define HW_COMPONENTS_DEVICES_REMOTE_CONTROL_ET08A_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "base.hpp"
#include "remote_control_base.hpp"
#include "sbus.hpp"
#include "system.hpp"

namespace hello_world
{
namespace remote_control
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START

struct ET08AChannelMap : public MemMgr {
  SbusCh J1 = kSbusCh1;
  SbusCh J2 = kSbusCh2;
  SbusCh J3 = kSbusCh3;
  SbusCh J4 = kSbusCh4;
  SbusCh SA = kSbusCh5;
  SbusCh SB = kSbusCh6;
  SbusCh SC = kSbusCh7;
  SbusCh SD = kSbusCh8;
  SbusCh LD = kSbusChInvalid;
  SbusCh RD = kSbusChInvalid;
};

struct ET08AConfig : public MemMgr {
  uint32_t offline_tick_thres = 100;
  ET08AChannelMap channel_map = ET08AChannelMap();
};

class ET08A : public Sbus
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  ET08A(void) = default;
  ET08A(const ET08AConfig &config);
  ET08A(const ET08A &) = default;
  ET08A &operator=(const ET08A &other);
  ET08A(ET08A &&other);
  ET08A &operator=(ET08A &&other);

  virtual ~ET08A(void) = default;

  /* 重载方法 */

  /**
   * @brief       解码数据
   * @param        len: 数据长度
   * @param        data: 数据指针
   * @retval       解码成功返回 true，否则返回 false
   * @note        None
   */
  virtual bool decode(size_t len, const uint8_t *data) override;

  /* 配置方法 */

  void init(const ET08AConfig &config);

  /* 数据修改与获取 */

  float J1(void) const { return J1_; }
  float J2(void) const { return J2_; }
  float J3(void) const { return J3_; }
  float J4(void) const { return J4_; }

  float LD(void) const { return LD_; }
  float RD(void) const { return RD_; }

  SwitchState SA(void) const { return SA_; }
  SwitchState SB(void) const { return SB_; }
  SwitchState SC(void) const { return SC_; }
  SwitchState SD(void) const { return SD_; }

  static float Ch2Pct(uint16_t ch_val)
  {
    float pct = (float)(ch_val - kChMidVal) / kChBias;
    return Bound(pct, -1.0f, 1.0f);
  }

  static SwitchState Ch2Sw(uint16_t ch_val)
  {
    if (ch_val == 0) {
      return SwitchState::kErr;
    } else if (ch_val < kChMidVal - kChBias / 2) {
      return SwitchState::kUp;
    } else if (ch_val > kChMidVal + kChBias / 2) {
      return SwitchState::kDown;
    } else {
      return SwitchState::kMid;
    }
  }

  static const uint16_t kChMidVal = 1024u;
  static const uint16_t kChBias = 670u;

 private:
  ET08AChannelMap channel_map_ = ET08AChannelMap();

  float J1_ = 0.0f;
  float J2_ = 0.0f;
  float J3_ = 0.0f;
  float J4_ = 0.0f;

  float LD_ = 0.0f;
  float RD_ = 0.0f;

  SwitchState SA_ = SwitchState::kErr;
  SwitchState SB_ = SwitchState::kErr;
  SwitchState SC_ = SwitchState::kErr;
  SwitchState SD_ = SwitchState::kErr;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace remote_control
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REMOTE_CONTROL_ET08A_HPP_ */
