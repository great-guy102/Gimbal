/**
 * @file      rfr_pkg_0x0301_inter_among_robots.cpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-02-19
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
 * | 1.0.0 | 2024-MM-DD | ZhouShichan | description |
 */
/* Define to prevent recursive inclusion -------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "rfr_pkg/rfr_pkg_0x0301_inter_among_robots.hpp"

namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint32_t InterAmongRobotsPackage::last_encode_tick_of_0x0301_ = 0;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

bool InterAmongRobotsPackage::encode(uint8_t *data)
{
  if (!isTxIntervalSatisfied()) {
    return false;
  }
  if (checkReceiverId(receiver_id_) && checkSenderId(sender_id_)) {
    InterConfig inter_config = {
        .data_cmd_id = getInterCmdId(),
        .sender_id = sender_id_,
        .receiver_id = receiver_id_,
    };
    memcpy(data, &inter_config, sizeof(InterConfig));
    encodeInterData(data + sizeof(InterConfig));
    uint32_t now_tick = getNowTickMs();
    last_encode_tick_ = now_tick;
    this->last_encode_tick_of_0x0301_ = now_tick;
    return true;
  }
  return false;
}

bool InterAmongRobotsPackage::isTxIntervalSatisfied(void) const
{
  return getNowTickMs() - last_encode_tick_of_0x0301_ >= getMinTxIntervalMs();
}
/* Private function definitions ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world