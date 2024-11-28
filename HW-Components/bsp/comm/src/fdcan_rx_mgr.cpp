/**
 *******************************************************************************
 * @file      : fdcan_rx_mgr.cpp
 * @brief     : FDCAN 接收管理器
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
#include "fdcan_rx_mgr.hpp"

/* 开启 FDCAN 才允许编译 */
#ifdef HAL_FDCAN_MODULE_ENABLED

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

FdCanRxMgr::FdCanRxMgr(FDCAN_HandleTypeDef *hfdcan, RxType rx_type)
    : RxMgr(), hfdcan_(hfdcan), rx_type_(rx_type)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(hfdcan != nullptr, "hfdcan is nullptr");
  HW_ASSERT(rx_type == RxType::kFifo0 || rx_type == RxType::kFifo1,
            "rx_type is invalid");
#pragma endregion
}

FdCanRxMgr &FdCanRxMgr::operator=(const FdCanRxMgr &other)
{
  if (this != &other) {
    RxMgr::operator=(other);

    hfdcan_ = other.hfdcan_;
    rx_type_ = other.rx_type_;
    status_ = other.status_;
    start_receive_ = other.start_receive_;
    decode_success_cnt_ = other.decode_success_cnt_;
  }

  return *this;
}

FdCanRxMgr::FdCanRxMgr(FdCanRxMgr &&other) : RxMgr(std::move(other))
{
  hfdcan_ = other.hfdcan_;
  rx_type_ = other.rx_type_;
  status_ = other.status_;
  start_receive_ = other.start_receive_;
  decode_success_cnt_ = other.decode_success_cnt_;

  other.hfdcan_ = nullptr;
}

FdCanRxMgr &FdCanRxMgr::operator=(FdCanRxMgr &&other)
{
  if (this != &other) {
    RxMgr::operator=(std::move(other));

    hfdcan_ = other.hfdcan_;
    rx_type_ = other.rx_type_;
    status_ = other.status_;
    start_receive_ = other.start_receive_;
    decode_success_cnt_ = other.decode_success_cnt_;

    other.hfdcan_ = nullptr;
  }

  return *this;
}

void FdCanRxMgr::startReceive(void)
{
  start_receive_ = true;

  uint32_t active_it = 0;
  if (rx_type_ == RxType::kFifo0) {
    active_it = FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF;
  } else {
    active_it = FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_BUS_OFF;
  }

  if (HAL_FDCAN_ActivateNotification(hfdcan_, active_it, 0) != HAL_OK) {
    SetBits(kFdCanRxStatusRecvStartFailed, status_);
  }
}

void FdCanRxMgr::stopReceive(void)
{
  start_receive_ = false;

  uint32_t inactive_it = 0;
  if (rx_type_ == RxType::kFifo0) {
    inactive_it = FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
  } else {
    inactive_it = FDCAN_IT_RX_FIFO1_NEW_MESSAGE;
  }

  HAL_FDCAN_DeactivateNotification(hfdcan_, inactive_it);
}

void FdCanRxMgr::init(FDCAN_HandleTypeDef *hfdcan, RxType rx_type)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(hfdcan != nullptr, "hfdcan is nullptr");
  HW_ASSERT(rx_type == RxType::kFifo0 || rx_type == RxType::kFifo1,
            "rx_type is invalid");
#pragma endregion

  hfdcan_ = hfdcan;
  rx_type_ = rx_type;

  status_ = kFdCanRxStatusOk;
  start_receive_ = false;
  decode_success_cnt_ = 0;
}

void FdCanRxMgr::filterInit(void)
{
  FDCAN_FilterTypeDef filter_config{
      .IdType = FDCAN_STANDARD_ID,
      .FilterIndex = 0,
      .FilterType = FDCAN_FILTER_MASK,
      .FilterID1 = 0x000,
      .FilterID2 = 0x000,
      .RxBufferIndex = 0,
      .IsCalibrationMsg = 0,
  };

  if (rx_type_ == RxType::kFifo0) {
    filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  } else {
    filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  }

  if (HAL_FDCAN_ConfigFilter(hfdcan_, &filter_config) != HAL_OK) {
    SetBits(kFdCanRxStatusFilterErr, status_);
  }
}

void FdCanRxMgr::configInterruptLines(ItLine it_line)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(it_line == ItLine::k0 || it_line == ItLine::k1,
            "it_line is invalid");
#pragma endregion

  uint32_t active_it = 0;
  if (rx_type_ == RxType::kFifo0) {
    active_it = FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
  } else {
    active_it = FDCAN_IT_RX_FIFO1_NEW_MESSAGE;
  }

  HAL_StatusTypeDef ret = HAL_OK;
  if (it_line == ItLine::k0) {
    ret = HAL_FDCAN_ConfigInterruptLines(
        hfdcan_, active_it, FDCAN_INTERRUPT_LINE0);
  } else {
    ret = HAL_FDCAN_ConfigInterruptLines(
        hfdcan_, active_it, FDCAN_INTERRUPT_LINE1);
  }

  if (ret != HAL_OK) {
    SetBits(kFdCanRxStatusItLinesConfigErr, status_);
  }
}

void FdCanRxMgr::rxFifoCallback(
    FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo_its)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(hfdcan != nullptr, "hfdcan is nullptr");
#pragma endregion

  if (hfdcan != hfdcan_ || !start_receive_) {
    return;
  }

  /* 判断是否接受到了新的数据 */
  if (rx_type_ == RxType::kFifo0 &&
      !(rx_fifo_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) {
    return;
  } else if (rx_type_ == RxType::kFifo1 &&
             !(rx_fifo_its & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)) {
    return;
  }

  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_buf[8];

  uint32_t fifo = 0;
  if (rx_type_ == RxType::kFifo0) {
    fifo = FDCAN_RX_FIFO0;
  } else {
    fifo = FDCAN_RX_FIFO1;
  }

  /* 获取FIFO里的所有数据 */
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan_, fifo) > 0) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, fifo, &rx_header, rx_buf) == HAL_OK) {
      /* 解码 */
      receive_cnt_++;
      decode_success_cnt_ += decode(
          rx_header.DataLength, rx_buf, rx_header.Identifier);
    } else {
      SetBits(kFdCanRxStatusGetRxMsgErr, status_);
    }
  }

  startReceive();
}

void FdCanRxMgr::errorStatusCallback(
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