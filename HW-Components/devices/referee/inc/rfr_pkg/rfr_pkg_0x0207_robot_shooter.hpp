/**
 * @file      rfr_pkg_0x0207_robot_shooter.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0207_ROBOT_SHOOTER_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0207_ROBOT_SHOOTER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "rfr_pkg_core.hpp"

namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/**
 * @enum BulletType
 * @brief 枚举弹丸类型枚举
 */
enum BulletType {
  kBulletTypeUnkown = 0u,  ///< 未知弹丸
  kBulletType17mm = 1u,    ///< 17mm弹丸
  kBulletType42mm = 2u,    ///< 42mm弹丸
};

/**
 * @enum ShooterId
 * @brief 枚举发射机构 ID 枚举
 */
enum ShooterId {
  kShooterIdUnkown = 0u,  ///< 未知发射机构
  kShooterId17mm1 = 1u,   ///< 第 1 个 17mm 发射机构
  kShooterId17mm2 = 2u,   ///< 第 2 个 17mm 发射机构
  kShooterId42mm = 3u,    ///< 42mm 发射机构
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/**
 * @struct RobotShooterData
 * @brief 机器人射击数据
 */
struct __REFEREE_PACKED RobotShooterData {
  uint8_t bullet_type;          ///< 弹丸类型，@see BulletType
  uint8_t shooter_id;           ///< 发射机构ID，@see ShooterId
  uint8_t launching_frequency;  ///< 弹丸射频，单位：Hz
  float bullet_speed;           ///< 弹丸初速度，单位：m/s
};
static_assert(sizeof(RobotShooterData) == 7, "RobotShooterData size error");
/** @class RobotShooterPackage
 * @brief 机器人射击数据包
 *
 * 数据说明：
 * - 命令码：0x0207
 * - 数据长度：7
 * - 发送频率：射击时触发发送
 * - 发送方/接收方：主控模块->对应机器人
 * - 所属数据链路：常规链路
 */
class RobotShooterPackage : public ProtocolRxPackage
{
 public:
  typedef RobotShooterData Data;

  virtual CmdId getCmdId(void) const override { return 0x0207; }
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

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0207_ROBOT_SHOOTER_HPP_ */
