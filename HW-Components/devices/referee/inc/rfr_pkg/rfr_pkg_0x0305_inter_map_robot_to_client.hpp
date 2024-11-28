/**
 * @file      rfr_pkg_0x0305_inter_map_robot_to_client.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0305_INTER_MAP_ROBOT_TO_CLIENT_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0305_INTER_MAP_ROBOT_TO_CLIENT_HPP_

/* Includes ------------------------------------------------------------------*/
#include "rfr_id.hpp"
#include "rfr_pkg_core.hpp"

namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/

enum class RobotType : uint8_t {
  kHero,       ///< 英雄机器人
  kEngineer,   ///< 工程机器人
  kInfantry3,  ///< 步兵机器人 3
  kInfantry4,  ///< 步兵机器人 4
  kInfantry5,  ///< 步兵机器人 5
  kSentry,     ///< 哨兵机器人
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @struct InterMapRobotToClientData
 * @brief 选手端小地图可接收机器人数据。
 * @note 当x、y超出边界时显示在对应边缘处，当x、y均为0时，视为未发送此机器人坐标，
 *       单位：cm
 */
struct __REFEREE_PACKED InterMapRobotToClientData {
  uint16_t hero_position_x;
  uint16_t hero_position_y;
  uint16_t engineer_position_x;
  uint16_t engineer_position_y;
  uint16_t infantry_3_position_x;
  uint16_t infantry_3_position_y;
  uint16_t infantry_4_position_x;
  uint16_t infantry_4_position_y;
  uint16_t infantry_5_position_x;
  uint16_t infantry_5_position_y;
  uint16_t sentry_position_x;
  uint16_t sentry_position_y;
};
static_assert(sizeof(InterMapRobotToClientData) == 24, "InterMapRobotToClientData size error");
/**
 * @class InterMapRobotToClientPackage
 * @brief 选手端小地图可接收机器人数据
 *
 * 雷达可通过常规链路向己方所有选手端发送对方机器人的坐标数据，该位置会在己方选手端小地
 * 图显示。
 *
 * 数据说明：
 *
 * - 命令码：0x0305
 * - 数据长度：24
 * - 发送频率：频率上限为 5Hz
 * - 发送方/接收方：雷达->服务器->己方所有选手端
 * - 所属数据链路：常规链路
 */
class InterMapRobotToClientPackage : public ProtocolTxPackage
{
 public:
  typedef InterMapRobotToClientData Data;
  typedef ids::RobotId RobotId;

  virtual CmdId getCmdId(void) const override { return 0x0305; }
  virtual DataLength getDataLength(void) const override { return sizeof(Data); }
  virtual uint32_t getMinTxIntervalMs(void) const override
  {
    return FREQ2INTERVAL(10);
  }

  const Data &getData(void) const { return data_; }

  bool setTargetPosition(RobotType type, uint16_t x, uint16_t y)
  {
    switch (type) {
      case RobotType::kHero:
        data_.hero_position_x = x;
        data_.hero_position_y = y;
        break;
      case RobotType::kEngineer:
        data_.engineer_position_x = x;
        data_.engineer_position_y = y;
        break;
      case RobotType::kInfantry3:
        data_.infantry_3_position_x = x;
        data_.infantry_3_position_y = y;
        break;
      case RobotType::kInfantry4:
        data_.infantry_4_position_x = x;
        data_.infantry_4_position_y = y;
        break;
      case RobotType::kInfantry5:
        data_.infantry_5_position_x = x;
        data_.infantry_5_position_y = y;
        break;
      case RobotType::kSentry:
        data_.sentry_position_x = x;
        data_.sentry_position_y = y;
        break;
      default:
        return false;
    }

    return true;
  }

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

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0305_INTER_MAP_ROBOT_TO_CLIENT_HPP_ */
