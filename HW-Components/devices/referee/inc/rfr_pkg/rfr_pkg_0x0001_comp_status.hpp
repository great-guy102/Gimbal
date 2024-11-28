/**
 * @file      rfr_pkg_0x0001_comp_status.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-25
 * @brief
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 *
 * @copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All
 * Rights Reserved.
 *
 * @attention
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2024-02-18 | ZhouShichan | 首次完成 |
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0001_COMP_STATUS_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0001_COMP_STATUS_HPP_

/* Includes ------------------------------------------------------------------*/
#include "rfr_pkg_core.hpp"

namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/**
 * @enum CompType
 * @brief 比赛类型枚举
 */
enum CompType : uint8_t {
  kCompTypeUnknown = 0u,  ///< 未知比赛类型
  kCompTypeRmuc = 1u,     ///< RoboMaster 机甲大师超级对抗赛
  kCompTypeRmutc = 2u,    ///< RoboMaster 机甲大师高校单项赛
  kCompTypeIcra = 3u,     ///< ICRA RoboMaster 高校人工智能挑战赛
  kCompTypeRmul3v3 = 4u,  ///< RoboMaster 机甲大师高校联盟赛 3V3 对抗
  kCompTypeRmul1v1 = 5u   ///< RoboMaster 机甲大师高校联盟赛步兵对抗
};

/**
 * @enum CompStage
 * @brief 比赛阶段枚举
 */
enum CompStage : uint8_t {
  kCompStageNothing = 0u,     ///< 未开始比赛
  kCompStageSetup = 1u,       ///< 准备阶段
  kCompStageInit = 2u,        ///< 十五秒裁判系统自检阶段
  kCompStageCountdown = 3u,   ///< 五秒倒计时
  kCompStageOngoing = 4u,     ///< 比赛中
  kCompStageCalcResults = 5u  ///< 比赛结算中
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/**
 * @struct CompStatusData
 * @brief 比赛状态数据
 */
struct __REFEREE_PACKED CompStatusData {
  uint8_t game_type : 4;       ///< 比赛类型 @see CompType
  uint8_t game_progress : 4;   ///< 当前比赛阶段 @see CompStage
  uint16_t stage_remain_time;  ///< 当前阶段剩余时间，单位：秒
  /*/< UNIX 时间戳，当机器人正确连接到裁判系统的 NTP 服务器后生效 */
  uint64_t sync_time_stamp;
};
static_assert(sizeof(CompStatusData) == 11, "CmpStatusData size error");

/** @class CompStatusPackage
 * @brief 比赛状态数据
 *
 * 数据说明：
 * - 命令码：0x0001
 * - 数据长度：11
 * - 发送频率：1Hz
 * - 发送方/接收方：服务器->全体机器人
 * - 所属数据链路：常规链路
 */
class CompStatusPackage : public ProtocolRxPackage
{
 public:
  typedef CompStatusData Data;

  virtual CmdId getCmdId(void) const override { return 0x0001; }
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
      last_decode_tick_ = tick::GetTickMs();
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

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0001_COMP_STATUS_HPP_ */
