/**
 * @file      rfr_pkg_0x0002_comp_result.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0002_COMP_RESULT_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0002_COMP_RESULT_HPP_

/* Includes ------------------------------------------------------------------*/
#include "rfr_pkg_core.hpp"

namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/**
 * @enum CompResults
 * @brief 比赛结果枚举
 */
enum CompResults : uint8_t {
  kCompResultDraw = 0u,          ///< 平局
  kCompResultRedTeamWins = 1u,   ///< 红方胜利
  kCompResultBlueTeamWins = 2u,  ///< 蓝方胜利
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @struct CompResultData
 * @brief 记录比赛结果数据
 */
struct __REFEREE_PACKED CompResultData {
  uint8_t result;  ///< 比赛结果 @see CompResults
};
static_assert(sizeof(CompResultData) == 1, "CmpResultData size error");
/** @class CompResultPackage
 * @brief 比赛结果数据包
 *
 * 数据说明：
 * - 命令码：0x0002
 * - 数据长度：1
 * - 发送频率：比赛结束触发发送
 * - 发送方/接收方：服务器->全体机器人
 * - 所属数据链路：常规链路
 */
class CompResultPackage : public ProtocolRxPackage
{
 public:
  typedef CompResultData Data;

  virtual CmdId getCmdId(void) const override { return 0x0002; }
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

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0002_COMP_RESULT_HPP_ */
