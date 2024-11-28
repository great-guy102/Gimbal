/**
 *******************************************************************************
 * @file      : can_rx_mgr.cpp
 * @brief     : CAN接收管理器
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2004-07-10      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  1. 需要在 STM32CubeMX 中开启 CAN 对应的接收中断，具体硬件配置详见 Wiki
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
#include "can_rx_mgr.hpp"

/* 开启 CAN 才允许编译 */
#ifdef HAL_CAN_MODULE_ENABLED

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

CanRxMgr::CanRxMgr(CAN_HandleTypeDef *hcan, RxType rx_type)
    : RxMgr(), hcan_(hcan), rx_type_(rx_type)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(hcan != nullptr, "hcan is nullptr");
  HW_ASSERT(rx_type == RxType::kFifo0 || rx_type == RxType::kFifo1,
            "rx_type is invalid");
#pragma endregion
}

CanRxMgr &CanRxMgr::operator=(const CanRxMgr &other)
{
  if (this != &other) {
    RxMgr::operator=(other);

    hcan_ = other.hcan_;
    rx_type_ = other.rx_type_;
    start_receive_ = other.start_receive_;
  }

  return *this;
}

CanRxMgr::CanRxMgr(CanRxMgr &&other)
    : RxMgr(std::move(other)),
      hcan_(other.hcan_),
      rx_type_(other.rx_type_),
      start_receive_(other.start_receive_)
{
  other.hcan_ = nullptr;
}

CanRxMgr &CanRxMgr::operator=(CanRxMgr &&other)
{
  if (this != &other) {
    RxMgr::operator=(std::move(other));

    hcan_ = other.hcan_;
    rx_type_ = other.rx_type_;
    start_receive_ = other.start_receive_;

    other.hcan_ = nullptr;
  }

  return *this;
}

void CanRxMgr::startReceive(void)
{
  start_receive_ = true;

  uint32_t active_it = 0;
  if (rx_type_ == RxType::kFifo0) {
    active_it = CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF;
  } else {
    active_it = CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF;
  }

  if (HAL_CAN_ActivateNotification(hcan_, active_it) != HAL_OK) {
    SetBits(kCanRxStatusRecvStartFailed, status_);
  }
}

void CanRxMgr::stopReceive(void)
{
  start_receive_ = false;

  uint32_t inactive_it = 0;
  if (rx_type_ == RxType::kFifo0) {
    inactive_it = CAN_IT_RX_FIFO0_MSG_PENDING;
  } else {
    inactive_it = CAN_IT_RX_FIFO1_MSG_PENDING;
  }

  HAL_CAN_DeactivateNotification(hcan_, inactive_it);
}

void CanRxMgr::init(CAN_HandleTypeDef *hcan, RxType rx_type)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(hcan != nullptr, "hcan is nullptr");
  HW_ASSERT(rx_type == RxType::kFifo0 || rx_type == RxType::kFifo1,
            "rx_type is invalid");
#pragma endregion

  hcan_ = hcan;
  rx_type_ = rx_type;

  status_ = kCanRxStatusOk;
  start_receive_ = false;
  decode_success_cnt_ = 0;
}

void CanRxMgr::filterInit(void)
{
  CAN_FilterTypeDef can_filter;

  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_16BIT;

  can_filter.FilterActivation = CAN_FILTER_ENABLE;
  can_filter.SlaveStartFilterBank = 14;

  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;

#ifdef CAN2
  if (hcan_->Instance == CAN2) {
    can_filter.FilterBank = 14;
  } else {
    can_filter.FilterBank = 0;
  }
#else
  can_filter.FilterBank = 0;
#endif

  if (rx_type_ == RxType::kFifo0) {
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  } else {
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO1;
  }

  if (HAL_CAN_ConfigFilter(hcan_, &can_filter) != HAL_OK) {
    SetBits(kCanRxStatusFilterErr, status_);
  }
}

void CanRxMgr::rxFifoMsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(hcan != nullptr, "hcan is nullptr");
#pragma endregion

  if (hcan != hcan_ || !start_receive_) {
    return;
  }

  uint32_t fifo = 0;
  if (rx_type_ == RxType::kFifo0) {
    fifo = CAN_RX_FIFO0;
  } else {
    fifo = CAN_RX_FIFO1;
  }

  /* 获取 FIFO 里的所有数据 */
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_buf[8];
  while (HAL_CAN_GetRxFifoFillLevel(hcan, fifo) > 0) {
    if (HAL_CAN_GetRxMessage(hcan, fifo, &rx_header, rx_buf) == HAL_OK) {
      /* 解码 */
      receive_cnt_++;
      decode_success_cnt_ += decode(rx_header.DLC, rx_buf, rx_header.StdId);
    } else {
      SetBits(kCanRxStatusGetRxMsgErr, status_);
    }
  }

  /* 重新开启接收 */
  startReceive();
}

void CanRxMgr::errorCallback(CAN_HandleTypeDef *hcan)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(hcan != nullptr, "hcan is nullptr");
#pragma endregion

  if (hcan == hcan_ && start_receive_) {
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