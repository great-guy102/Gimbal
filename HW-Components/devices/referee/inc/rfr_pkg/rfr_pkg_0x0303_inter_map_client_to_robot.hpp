/**
 * @file      rfr_pkg_0x0303_inter_map_client_to_robot.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0303_INTER_MAP_CLIENT_TO_ROBOT_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0303_INTER_MAP_CLIENT_TO_ROBOT_HPP_

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
 * @struct InterMapClientToRobotData
 * @brief 选手端小地图交互数据
 */
struct __REFEREE_PACKED InterMapClientToRobotData {
  /*/< 目标位置 x 轴坐标，单位 m @note 当发送目标机器人 ID 时，该值为 0 */
  float target_position_x;
  /*/< 目标位置 y 轴坐标，单位 m @note 当发送目标机器人 ID 时，该值为 0 */
  float target_position_y;
  /*/< 云台手按下的键盘按键通用键值 @note 无按键按下，则为 0 */
  uint8_t cmd_keyboard;
  uint8_t target_robot_id;  ///< 对方机器人 ID @note 当发送坐标数据时，该值为 0
  /*/< 信息来源 ID @note 信息来源的 ID，ID 对应关系详见附录或 rfr_id.hpp */
  uint16_t cmd_source;
};
static_assert(sizeof(InterMapClientToRobotData) == 12, "InterMapClientToRobotData size error");
/** ! 信息来源 ID 的描述上是 2 字节大小但给出的数据结构体中是 1 字节大小，且该结构体与
 * 总结表中的数据长度不一致 */

/**
 * @class InterMapClientToRobotPackage
 * @brief 选手端小地图交互数据
 *
 * - 云台手可通过选手端大地图向机器人发送固定数据。
 *
 *     命令码为 0x0303，触发时发送，两次发送间隔不得低于 0.5 秒。
 *
 *     **发送方式一：**
 *     1. 点击己方机器人头像；
 *     2. （可选）按下一个键盘按键或点击对方机器人头像；
 *     3. 点击小地图任意位置。该方式向己方选定的机器人发送地图坐标数据，若点击对方机器
 *        人头像，则以目标机器人 ID 代替坐标数据。
 *     **发送方式二：**
 *     1.（可选）按下一个键盘按键或点击对方机器人头像；
 *     2.点击小地图任意位置。该方式向己方所有机器人发送地图坐标数据，若点击对方机器人头
 *       像，则以目标机器人 ID 代替坐标数据。
 *
 * - 选择半自动控制方式的机器人对应的操作手可通过选手端大地图向机器人发送固定数据。
 *
 *     命令码为 0x0303，触发时发送，两次发送间隔不得低于 3 秒。
 *
 *     **发送方式：**
 *     1. （可选）按下一个键盘按键或点击对方机器人头像；
 *     2. 点击小地图任意位置。该方式向操作手对应的机器人发送地图坐标数据，若点击对方机
 *        器人头像，则以目标机器人 ID 代替坐标数据。
 *
 *     一台选择半自动控制方式的机器人既可以接收云台手发送的信息，也可以接收对应操作手的
 *     信息。两种信息的来源将在下表中“信息来源”中进行区别。
 *
 * - 为降低机器人串口接收设备的偶发不稳定性对通信的影响， 0x0303 协议的发送机制有所特殊
 *   处理，具体如下：选手端触发 1 次发送后，服务器将以 100ms 的间隔向机器人额外发送 4
 *   次，共 5 次。此后，直到下一次选手端触发发送前，服务器都将以1Hz的频率持续定频发送最
 *   近一次的包。触发时的连续发送和 1Hz 定频发送计时相互独立。队伍需关注多次收到重复协
 *   议内容的处理方式。
 *
 * 数据说明：
 *
 * - 命令码：0x0303
 * - 数据长度：12
 * - 发送频率：选手端触发发送，发送间隔不低于 0.5 秒或 3 秒
 * - 发送方/接收方：选手端点击->服务器->发送方选择的己方机器人
 * - 所属数据链路：常规链路
 *
 * @attention 在《串口协议v1.6.1> P7 表2-1 中，0x0303 的 数据段长度 字段为 15 ，与
 *            P30 表3-1 中的 大小 字段和结构体的定义不一致，此处以结构体大小为准。
 */
class InterMapClientToRobotPackage : public ProtocolRxPackage
{
 public:
  typedef InterMapClientToRobotData Data;

  virtual CmdId getCmdId(void) const override { return 0x0303; }
  virtual DataLength getDataLength(void) const override { return sizeof(Data); }
  virtual uint32_t getMaxRxIntervalMs(void) const override { return 500; }

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

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0303_INTER_MAP_CLIENT_TO_ROBOT_HPP_ */
