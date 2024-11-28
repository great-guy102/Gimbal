/**
 *******************************************************************************
 * @file      : can_tx_mgr.cpp
 * @brief     : CAN发送管理器
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2004-07-10      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  1. 需要在STM32CubeMX中开启CAN的发送中断，具体硬件配置详见 Wiki
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
/* Includes ------------------------------------------------------------------*/
#include "can_tx_mgr.hpp"

/* 开启 CAN 才允许编译 */
#ifdef HAL_CAN_MODULE_ENABLED

#include <cstring>

#include "assert.hpp"
#include "base.hpp"
#include "system.hpp"

namespace hello_world
{
namespace comm
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

HW_OPTIMIZE_O2_START

CanTxMgr::CanTxMgr(CAN_HandleTypeDef *hcan) : TxMgr(), hcan_(hcan)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(hcan != nullptr, "hcan is nullptr");
#pragma endregion
}

CanTxMgr &CanTxMgr::operator=(const CanTxMgr &other)
{
  if (this != &other) {
    TxMgr::operator=(other);

    hcan_ = other.hcan_;
  }

  return *this;
}

CanTxMgr::CanTxMgr(CanTxMgr &&other) : TxMgr(std::move(other))
{
  hcan_ = other.hcan_;
  other.hcan_ = nullptr;
}

CanTxMgr &CanTxMgr::operator=(CanTxMgr &&other)
{
  if (this != &other) {
    TxMgr::operator=(std::move(other));

    hcan_ = other.hcan_;
    other.hcan_ = nullptr;
  }

  return *this;
}

void CanTxMgr::startTransmit(void)
{
  start_transmit_ = true;
  HAL_CAN_ActivateNotification(
      hcan_, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_ERROR | CAN_IT_BUSOFF);

  /* 防止重入 */
  if (IsBitsSet(kCanTxStatusBusy, status_)) {
    return;
  }

  SetBits(kCanTxStatusBusy, status_);

  if (getTxMailboxesFreeLevel() == 0) {
    SetBits(kCanTxStatusTxMailboxFull, status_);
    ClearBits(kCanTxStatusBusy, status_);
    return;
  }

  uint8_t tx_buf[8];
  uint32_t tx_mailbox;
  CAN_TxHeaderTypeDef tx_header = {
      .StdId = 0,
      .ExtId = 0,
      .IDE = CAN_ID_STD,
      .RTR = CAN_RTR_DATA,
      .DLC = 8,
      .TransmitGlobalTime = DISABLE,
  };

  /* 当有空邮箱且有需要发送的数据时进行发送 */
  while (getTxMailboxesFreeLevel() > 0 && getRemainMsgNum() > 0) {
    size_t tmp_len = tx_header.DLC;
    size_t encode_success_num = encode(tmp_len, tx_buf, tx_header.StdId);
    if (encode_success_num != 0) {
      encode_success_cnt_ += encode_success_num;
      /* 编码成功后发送数据 */
      if (HAL_CAN_AddTxMessage(hcan_, &tx_header, tx_buf, &tx_mailbox) ==
          HAL_OK) {
        transmit_cnt_++;
        setTransmitterFinished(tx_header.StdId);
      } else {
        SetBits(kCanTxStatusTxErr, status_);
        break;
      }
    }
  }

  ClearBits(kCanTxStatusBusy, status_);
}

void CanTxMgr::stopTransmit(void)
{
  start_transmit_ = false;

  uint32_t tx_mailboxs = CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2;
  HAL_CAN_AbortTxRequest(hcan_, tx_mailboxs);
  HAL_CAN_DeactivateNotification(hcan_, CAN_IT_TX_MAILBOX_EMPTY);
}

void CanTxMgr::init(CAN_HandleTypeDef *hcan)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(hcan != nullptr, "hcan is nullptr");
#pragma endregion

  hcan_ = hcan;

  status_ = kCanTxStatusOk;
  start_transmit_ = false;
  encode_success_cnt_ = 0;
}

void CanTxMgr::txMailboxCompleteCallback(CAN_HandleTypeDef *hcan)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(hcan != nullptr, "hcan is nullptr");
#pragma endregion

  if (hcan == hcan_ && start_transmit_) {
    startTransmit();
  }
}

void CanTxMgr::errorCallback(CAN_HandleTypeDef *hcan)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(hcan != nullptr, "hcan is nullptr");
#pragma endregion

  if (hcan == hcan_ && start_transmit_) {
    /* 错误为无人接收时发送下一个报文 */
    if (hcan_->ErrorCode & HAL_CAN_ERROR_TX_TERR0 ||
        hcan_->ErrorCode & HAL_CAN_ERROR_TX_TERR1 ||
        hcan_->ErrorCode & HAL_CAN_ERROR_TX_TERR2) {
      startTransmit();
    }
    if (hcan_->ErrorCode & HAL_CAN_ERROR_BOF) {
      HAL_CAN_Init(hcan_);
      HAL_CAN_Start(hcan_);
    }
  }
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace comm
}  // namespace hello_world

#endif /* HAL_CAN_MODULE_ENABLED */
