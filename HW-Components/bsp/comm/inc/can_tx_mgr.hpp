/**
 *******************************************************************************
 * @file      : can_tx_mgr.hpp
 * @brief     : CAN 发送管理器
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2004-07-10      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  1. 需要在 STM32CubeMX 中开启 CAN 的发送中断，具体硬件配置详见 Wiki
 *  2. 由于内部使用了硬件句柄，因此如果计划将实例作为全局变量时（全局变量初始化时对应的
 *  硬件句柄可能会还未初始化完毕），建议采取一下方法：
 *    1）声明指针，后续通过 `new` 的方式进行初始化
 *    2）声明指针，后续通过返回函数（CreateXXXIns）中的静态变量（因为该变量只有在第一
 *    次调用该函数时才会运行初始化程序）进行初始化
 *    3）使用无参构造函数，后续调用 `init` 方法进行初始化
 *    4）使用无参构造函数，后续使用拷贝赋值函数或是移动赋值函数进行初始化
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_BSP_COMM_CAN_TX_MGR_HPP_
#define HW_COMPONENTS_BSP_COMM_CAN_TX_MGR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "stm32_hal.hpp"

/* 开启 CAN 才允许编译 */
#ifdef HAL_CAN_MODULE_ENABLED

#include "tx_mgr.hpp"

namespace hello_world
{
namespace comm
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START

enum CanTxStatus {
  kCanTxStatusOk = 0,                  ///< 正常
  kCanTxStatusBusy = 1 << 0,           ///< 忙
  kCanTxStatusTxMailboxFull = 1 << 1,  ///< 发送邮箱满
  kCanTxStatusTxErr = 1 << 2,          ///< 发送错误
};

class CanTxMgr : public TxMgr
{
 public:
  typedef CanTxStatus Status;

  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  CanTxMgr(void) = default;
  /**
   * @brief       构造函数
   * @param        hcan: CAN 句柄
   * @retval       None
   * @note        None
   */
  CanTxMgr(CAN_HandleTypeDef *hcan);
  CanTxMgr(const CanTxMgr &) = default;
  CanTxMgr &operator=(const CanTxMgr &other);
  CanTxMgr(CanTxMgr &&other);
  CanTxMgr &operator=(CanTxMgr &&other);

  virtual ~CanTxMgr(void) = default;

  /* 重载方法 */

  /**
   * @brief       开始 CAN 发送
   * @retval       None
   * @note        方法用于在所有发送端添加完毕后开启发送，开启后相同 ID 的发送端将被依
   *              次调用 encode 方法对同一条报文进行编码，完成后再将报文通过外设发出，
   *              为适配各个发送端可能有不同的发送频率或是交替发送的特殊需求，因此每次
   *              要发送数据时都需要先调用 setTransmitterNeedToTransmit 方法再调用
   *              发送开启方法，同时对于某个 ID 对应的报文，只有所有发送端编码均成功，
   *              发送管理器才会认为该报文处于可发送状态
   */
  virtual void startTransmit(void) override;

  /**
   * @brief       停止 CAN 发送
   * @retval       None
   * @note        None
   */
  virtual void stopTransmit(void) override;

  /* 配置方法 */

  /**
   * @brief       初始化 CAN 发送管理器，使用默认构造函数后请务必调用此函数
   * @param        hcan: CAN 句柄
   * @retval       None
   * @note        None
   */
  void init(CAN_HandleTypeDef *hcan);

  /* 回调函数 */

  /**
   * @brief       CAN 发送邮箱完成回调
   * @param        hcan: CAN 句柄
   * @retval       None
   * @note        请将此函数放置于所有的 HAL_CAN_TxMailboxXCompleteCallback 中（X
   *              为 0、1、2），用于在有空邮箱时发送下一个报文
   */
  void txMailboxCompleteCallback(CAN_HandleTypeDef *hcan);

  /**
   * @brief       CAN 错误回调
   * @param        hcan: CAN 句柄
   * @retval       None
   * @note        请将此函数放置于所有的 HAL_CAN_ErrorCallback 中，用于在接收方无回
   *              应时发下一个报文，同时在总线错误时重启 CAN
   */
  void errorCallback(CAN_HandleTypeDef *hcan);

  /* 数据修改与获取 */

  Status status(void) const { return status_; }

  uint32_t getTxMailboxesFreeLevel(void) const
  {
    return HAL_CAN_GetTxMailboxesFreeLevel(hcan_);
  }

 private:
  /* 硬件相关 */

  CAN_HandleTypeDef *hcan_ = nullptr;

  /* 发送状态相关 */

  Status status_ = kCanTxStatusOk;
  bool start_transmit_ = false;
  uint32_t encode_success_cnt_ = 0;
  uint32_t transmit_cnt_ = 0;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace comm
}  // namespace hello_world

#endif /* HAL_CAN_MODULE_ENABLED */

#endif /* HW_COMPONENTS_BSP_COMM_CAN_TX_MGR_HPP_ */
