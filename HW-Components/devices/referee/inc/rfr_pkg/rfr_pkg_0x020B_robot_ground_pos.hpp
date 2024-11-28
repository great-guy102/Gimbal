/**
 * @file      rfr_pkg_0x020B_robot_ground_pos.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-25
 * @brief
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 * | 1.0.0 | 2024-MM-DD | ZhouShichan | description |
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X020B_ROBOT_GROUND_POS_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X020B_ROBOT_GROUND_POS_HPP_

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
 *  @struct RobotsGroundPosData
 *  @brief  地面机器人的位置数据
 *
 *  场地围挡在红方补给站附近的交点为坐标原点，沿场地长边向蓝方为 X 轴正方向，沿场地短边
 *  向红方停机坪为 Y 轴正方向。
 */
struct __REFEREE_PACKED RobotsGroundPosData {
  float hero_x;        ///< 己方英雄机器人位置 x 轴坐标，单位：m
  float hero_y;        ///< 己方英雄机器人位置 y 轴坐标，单位：m
  float engineer_x;    ///< 己方工程机器人位置 x 轴坐标，单位：m
  float engineer_y;    ///< 己方工程机器人位置 y 轴坐标，单位：m
  float standard_3_x;  ///< 己方 3 号步兵机器人位置 x 轴坐标，单位：m
  float standard_3_y;  ///< 己方 3 号步兵机器人位置 y 轴坐标，单位：m
  float standard_4_x;  ///< 己方 4 号步兵机器人位置 x 轴坐标，单位：m
  float standard_4_y;  ///< 己方 4 号步兵机器人位置 y 轴坐标，单位：m
  float standard_5_x;  ///< 己方 5 号步兵机器人位置 x 轴坐标，单位：m
  float standard_5_y;  ///< 己方 5 号步兵机器人位置 y 轴坐标，单位：m
};
static_assert(sizeof(RobotsGroundPosData) == 40, "RobotsGroundPosData size error");
/** @class RobotsGroundPosPackage
 * @brief 地面机器人位置数据包
 *
 * 数据说明：
 * - 命令码：0x020B
 * - 数据长度：40
 * - 发送频率：1Hz
 * - 发送方/接收方：服务器->哨兵机器人
 * - 所属数据链路：常规链路
 */
class RobotsGroundPosPackage : public ProtocolRxPackage
{
 public:
  typedef RobotsGroundPosData Data;

  virtual CmdId getCmdId(void) const override { return 0x020B; }
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

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X020B_ROBOT_GROUND_POS_HPP_ */
