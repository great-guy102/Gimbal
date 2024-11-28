/**
 * @file      rfr_pkg_0x0003_comp_robots_hp.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0003_COMP_ROBOTS_HP_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0003_COMP_ROBOTS_HP_HPP_

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
 * @struct CompRobotsHpData
 * @brief 所有机器人的血量数据
 */
struct __REFEREE_PACKED CompRobotsHpData {
  /*/< 红 1 英雄机器人血量。若该机器人未上场或者被罚下，则血量为 0 */
  uint16_t red_1_robot_HP;
  /*/< 红 2 工程机器人血量。若该机器人未上场或者被罚下，则血量为 0 */
  uint16_t red_2_robot_HP;
  /*/< 红 3 步兵机器人血量。若该机器人未上场或者被罚下，则血量为 0 */
  uint16_t red_3_robot_HP;
  /*/< 红 4 步兵机器人血量。若该机器人未上场或者被罚下，则血量为 0 */
  uint16_t red_4_robot_HP;
  /*/< 红 5 步兵机器人血量。若该机器人未上场或者被罚下，则血量为 0 */
  uint16_t red_5_robot_HP;
  /*/< 红 7 哨兵机器人血量。若该机器人未上场或者被罚下，则血量为 0 */
  uint16_t red_7_robot_HP;
  uint16_t red_outpost_HP;  ///< 红方前哨站血量
  uint16_t red_base_HP;     ///< 红方基地血量
  /*/< 蓝 1 英雄机器人血量。若该机器人未上场或者被罚下，则血量为 0 */
  uint16_t blue_1_robot_HP;
  /*/< 蓝 2 工程机器人血量。若该机器人未上场或者被罚下，则血量为 0 */
  uint16_t blue_2_robot_HP;
  /*/< 蓝 3 步兵机器人血量。若该机器人未上场或者被罚下，则血量为 0 */
  uint16_t blue_3_robot_HP;
  /*/< 蓝 4 步兵机器人血量。若该机器人未上场或者被罚下，则血量为 0 */
  uint16_t blue_4_robot_HP;
  /*/< 蓝 5 步兵机器人血量。若该机器人未上场或者被罚下，则血量为 0 */
  uint16_t blue_5_robot_HP;
  /*/< 蓝 7 哨兵机器人血量。若该机器人未上场或者被罚下，则血量为 0 */
  uint16_t blue_7_robot_HP;
  uint16_t blue_outpost_HP;  ///< 蓝方前哨站血量
  uint16_t blue_base_HP;     ///< 蓝方基地血量
};
static_assert(sizeof(CompRobotsHpData) == 32, "CompRobotsHpData size error");

/** @class CompRobotsHpPackage
 * @brief 比赛机器人血量数据包
 *
 * 数据说明：
 * - 命令码：0x0003
 * - 数据长度：32
 * - 发送频率：3Hz
 * - 发送方/接收方：服务器->全体机器人
 * - 所属数据链路：常规链路
 */
class CompRobotsHpPackage : public ProtocolRxPackage
{
 public:
  typedef CompRobotsHpData Data;

  virtual CmdId getCmdId(void) const override { return 0x0003; }
  virtual DataLength getDataLength(void) const override { return sizeof(Data); }
  virtual uint32_t getMaxRxIntervalMs(void) const override { 
    return FREQ2INTERVAL(3); }

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

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0003_COMP_ROBOTS_HP_HPP_ */
