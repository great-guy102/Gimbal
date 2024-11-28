/**
 * @file      rfr_pkg_0x0206_robot_hurt.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0206_ROBOT_HURT_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0206_ROBOT_HURT_HPP_

/* Includes ------------------------------------------------------------------*/
#include "rfr_pkg_core.hpp"

namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/**
 * @enum HpDeductionReason
 * @brief 扣除血量的原因枚举
 */
enum HpDeductionReason {
  kHpDeductionReasonArmorHit = 0u,          ///< 装甲被弹丸攻击扣血
  kHpDeductionReasonModuleOffline,          ///< 裁判系统重要模块离线扣血
  kHpDeductionReasonExceedingBulletSpeed,   ///< 射击初速度超限扣血
  kHpDeductionReasonExceedingBarrelHeat,    ///< 枪口热量超限扣血
  kHpDeductionReasonExceedingChassisPower,  ///< 底盘功率超限扣血
  kHpDeductionReasonArmorCollision,         ///< 装甲模块受到撞击扣血
};
/* Exported types ------------------------------------------------------------*/

/**
 * @struct RobotHurtData
 * @brief 所受伤害状态数据
 */
struct __REFEREE_PACKED RobotHurtData {
  /**
   * 当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，这四比特组成的数值为
   * 装甲模块或测速模块的ID编号；当其他原因导致扣血时，这个数值为0
   */
  uint8_t module_id : 4;
  uint8_t hp_deduction_reason : 4;  ///< 扣血原因，@see HpDeductionReason
};
static_assert(sizeof(RobotHurtData) == 1, "RobotHurtData size error");
/** @class RobotHurtPackage
 * @brief 所受伤害状态数据包
 * @note 受伤害情况为机器人裁判系统本地判定，即时发送，但实际是否受到对应伤害受规
则条例影响，请以服务器最终判定为准
 *
 * 数据说明：
 * - 命令码：0x0206
 * - 数据长度：1
 * - 发送频率：伤害发生时触发发送
 * - 发送方/接收方：主控模块->对应机器人
 * - 所属数据链路：常规链路
 */
class RobotHurtPackage : public ProtocolRxPackage
{
 public:
  typedef RobotHurtData Data;

  virtual CmdId getCmdId(void) const override { return 0x0206; }
  virtual DataLength getDataLength(void) const override { return sizeof(Data); }
  virtual uint32_t getMaxRxIntervalMs(void) const override
  {
    return FREQ2INTERVAL(0);
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

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0206_ROBOT_HURT_HPP_ */
