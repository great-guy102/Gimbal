/**
 *******************************************************************************
 * @file      : fdcan_tx_mgr.cpp
 * @brief     : FDCAN 发送管理器
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-07-11      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  1. 需要在 STM32CubeMX 中开启 FDCAN 的中断，具体硬件配置详见 Wiki
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
#include "fdcan_tx_mgr.hpp"

/* 开启 FDCAN 才允许编译 */
#ifdef HAL_FDCAN_MODULE_ENABLED

#include <cstring>

#include "assert.hpp"
#include "base.hpp"

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

FdCanTxMgr::FdCanTxMgr(FDCAN_HandleTypeDef *hfdcan) : TxMgr(), hfdcan_(hfdcan)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(hfdcan != nullptr, "hfdcan is nullptr");
#pragma endregion
}

FdCanTxMgr &FdCanTxMgr::operator=(const FdCanTxMgr &other)
{
  if (this != &other) {
    hfdcan_ = other.hfdcan_;
    status_ = other.status_;
    start_transmit_ = other.start_transmit_;
    encode_success_cnt_ = other.encode_success_cnt_;
  }

  return *this;
}

FdCanTxMgr &FdCanTxMgr::operator=(FdCanTxMgr &&other)
{
  if (this != &other) {
    hfdcan_ = other.hfdcan_;
    status_ = other.status_;
    start_transmit_ = other.start_transmit_;
    encode_success_cnt_ = other.encode_success_cnt_;

    other.hfdcan_ = nullptr;
  }

  return *this;
}

void FdCanTxMgr::startTransmit(void)
{
  start_transmit_ = true;
  HAL_FDCAN_ActivateNotification(
      hfdcan_, FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_BUS_OFF, 0);

  /* 避免重入 */
  if (IsBitsSet(kFdCanTxStatusBusy, status_)) {
    return;
  }

  SetBits(kFdCanTxStatusBusy, status_);

  if (getTxMailboxesFreeLevel() == 0) {
    SetBits(kFdCanTxStatusTxMailboxFull, status_);
    ClearBits(kFdCanTxStatusBusy, status_);
    return;
  }

  uint8_t tx_buf[8];
  FDCAN_TxHeaderTypeDef tx_header{
      .Identifier = 0,
      .IdType = FDCAN_STANDARD_ID,
      .TxFrameType = FDCAN_DATA_FRAME,
      .DataLength = FDCAN_DLC_BYTES_8,
      .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
      .BitRateSwitch = FDCAN_BRS_OFF,
      .FDFormat = FDCAN_CLASSIC_CAN,
      .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
      .MessageMarker = 0,
  };

  /* 当有空邮箱且有需要发送的数据时进行发送 */
  while (getTxMailboxesFreeLevel() > 0 && getRemainMsgNum() > 0) {
    size_t tmp_len = 8;
    uint32_t encode_success_num = encode(tmp_len, tx_buf, tx_header.Identifier);
    if (encode_success_num != 0) {
      encode_success_cnt_ += encode_success_num;
      /* 编码成功后发送数据 */
      if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &tx_header, tx_buf) ==
          HAL_OK) {
        transmit_cnt_++;
        setTransmitterFinished(tx_header.Identifier);
      } else {
        SetBits(kFdCanTxStatusTxErr, status_);
        break;
      }
    }
  }

  ClearBits(kFdCanTxStatusBusy, status_);
}

void FdCanTxMgr::stopTransmit(void)
{
  start_transmit_ = false;

  HAL_FDCAN_DeactivateNotification(hfdcan_, FDCAN_IT_TX_FIFO_EMPTY);
}

void FdCanTxMgr::init(FDCAN_HandleTypeDef *hfdcan)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(hfdcan != nullptr, "hfdcan is nullptr");
#pragma endregion

  hfdcan_ = hfdcan;

  status_ = kFdCanTxStatusOk;
  start_transmit_ = false;
  encode_success_cnt_ = 0;
}

void FdCanTxMgr::configInterruptLines(ItLine it_line)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(it_line == ItLine::k0 || it_line == ItLine::k1,
            "it_line is invalid");
#pragma endregion

  HAL_StatusTypeDef ret = HAL_OK;
  if (it_line == ItLine::k0) {
    ret = HAL_FDCAN_ConfigInterruptLines(
        hfdcan_, FDCAN_IT_TX_FIFO_EMPTY, FDCAN_INTERRUPT_LINE0);
  } else {
    ret = HAL_FDCAN_ConfigInterruptLines(
        hfdcan_, FDCAN_IT_TX_FIFO_EMPTY, FDCAN_INTERRUPT_LINE1);
  }

  if (ret != HAL_OK) {
    SetBits(kFdCanTxStatusItLinesConfigErr, status_);
  }
}

void FdCanTxMgr::txFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(hfdcan != nullptr, "hfdcan is nullptr");
#pragma endregion

  if (hfdcan == hfdcan_ && start_transmit_) {
    startTransmit();
  }
}

void FdCanTxMgr::errorStatusCallback(
    FDCAN_HandleTypeDef *hfdcan, uint32_t error_status_its)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(hfdcan != nullptr, "hfdcan is nullptr");
#pragma endregion

  if (hfdcan == hfdcan_ && (error_status_its & FDCAN_IT_BUS_OFF)) {
    FDCAN_ProtocolStatusTypeDef protocol_status;
    HAL_FDCAN_GetProtocolStatus(hfdcan_, &protocol_status);
    if (protocol_status.BusOff) {
      HAL_FDCAN_Init(hfdcan_);
      HAL_FDCAN_Start(hfdcan_);
    }
  }
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace comm
}  // namespace hello_world

#endif /* HAL_FDCAN_MODULE_ENABLED */
