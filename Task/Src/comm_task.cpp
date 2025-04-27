/**
 *******************************************************************************
 * @file      :comm_task.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "comm_task.hpp"

// hal
#include "can.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"

// HW-Components
#include "tick.hpp"

// custom
#include "ins_all.hpp"

using hello_world::comm::CanRxMgr;
using hello_world::comm::CanTxMgr;
using hello_world::comm::UartRxMgr;
using hello_world::comm::UartTxMgr;
using hello_world::remote_control::DT7;
/* Private macro -------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// rx communication components objects
static CanRxMgr *can1_rx_mgr_ptr = nullptr;
static CanTxMgr *can1_tx_mgr_ptr = nullptr;

static CanRxMgr *can2_rx_mgr_ptr = nullptr;
static CanTxMgr *can2_tx_mgr_ptr = nullptr;

static UartRxMgr *vision_rx_mgr_ptr = nullptr;
static UartTxMgr *vision_tx_mgr_ptr = nullptr;

static UartRxMgr *rc_rx_mgr_ptr = nullptr;

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void PrivatePointerInit(void);
static void CommAddReceiver(void);
static void CommAddTransmitter(void);
static void CommHardWareInit(void);

/* Exported function definitions ---------------------------------------------*/

void CommTask(void) {
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr",
            can1_tx_mgr_ptr);
  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr",
            can2_tx_mgr_ptr);
  HW_ASSERT(vision_tx_mgr_ptr != nullptr, "vision_tx_mgr_ptr is nullptr",
            vision_tx_mgr_ptr);
  can1_tx_mgr_ptr->startTransmit();
  can2_tx_mgr_ptr->startTransmit();
  vision_tx_mgr_ptr->startTransmit();
};

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  HW_ASSERT(can1_rx_mgr_ptr != nullptr, "can1_rx_mgr_ptr is nullptr",
            can1_rx_mgr_ptr);
  HW_ASSERT(can2_rx_mgr_ptr != nullptr, "can2_rx_mgr_ptr is nullptr",
            can2_rx_mgr_ptr);
  can1_rx_mgr_ptr->rxFifoMsgPendingCallback(hcan);
  can2_rx_mgr_ptr->rxFifoMsgPendingCallback(hcan);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  HW_ASSERT(can1_rx_mgr_ptr != nullptr, "can1_rx_mgr_ptr is nullptr",
            can1_rx_mgr_ptr);
  HW_ASSERT(can2_rx_mgr_ptr != nullptr, "can2_rx_mgr_ptr is nullptr",
            can2_rx_mgr_ptr);
  can1_rx_mgr_ptr->rxFifoMsgPendingCallback(hcan);
  can2_rx_mgr_ptr->rxFifoMsgPendingCallback(hcan);
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr",
            can1_tx_mgr_ptr);
  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr",
            can2_tx_mgr_ptr);
  can1_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
  can2_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr",
            can1_tx_mgr_ptr);
  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr",
            can2_tx_mgr_ptr);
  can1_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
  can2_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr",
            can1_tx_mgr_ptr);
  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr",
            can2_tx_mgr_ptr);
  can1_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
  can2_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr",
            can1_tx_mgr_ptr);
  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr",
            can2_tx_mgr_ptr);
  can1_tx_mgr_ptr->errorCallback(hcan);
  can2_tx_mgr_ptr->errorCallback(hcan);
}

uint32_t uart_rx_tick = 0;
uint32_t uart_rx_cb_in = 0;
float uart3_uticks = 0;
float uart6_uticks = 0;
uint32_t uart3_rx_cnt = 0;
uint32_t uart6_rx_cnt = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  uart_rx_tick = hello_world::tick::GetTickMs();
  uart_rx_cb_in++;

  // 遥控器
  if (huart == &huart3) {
    uart3_rx_cnt++;
    uint32_t tick_start = __HAL_TIM_GET_COUNTER(&htim2);
    HW_ASSERT(rc_rx_mgr_ptr != nullptr, "rc_rx_mgr_ptr is nullptr",
              rc_rx_mgr_ptr);
    rc_rx_mgr_ptr->rxEventCallback(huart, Size);
    HAL_IWDG_Refresh(&hiwdg);
    uint32_t tick_end = __HAL_TIM_GET_COUNTER(&htim2);
    uart3_uticks = (tick_end - tick_start) / (84.0f * 1e3);
    uart3_rx_cnt--;
  }
  // 视觉
  else if (huart == &huart6) {
    uart6_rx_cnt++;
    uint32_t tick_start = __HAL_TIM_GET_COUNTER(&htim2);
    HW_ASSERT(vision_rx_mgr_ptr != nullptr, "vision_rx_mgr_ptr is nullptr",
              vision_rx_mgr_ptr);
    vision_rx_mgr_ptr->rxEventCallback(huart, Size);
    uint32_t tick_end = __HAL_TIM_GET_COUNTER(&htim2);
    uart6_uticks = (tick_end - tick_start) / (84.0f * 1e3);
    uart6_rx_cnt--;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  // 遥控器
  if (huart == &huart3) {
    rc_rx_mgr_ptr->startReceive();
  }
}

void CommTaskInit(void) {
  PrivatePointerInit();
  CommAddReceiver();
  CommAddTransmitter();
  CommHardWareInit();
};

static void PrivatePointerInit(void) {
  can1_rx_mgr_ptr = GetCan1RxMgr();
  can1_tx_mgr_ptr = GetCan1TxMgr();

  can2_rx_mgr_ptr = GetCan2RxMgr();
  can2_tx_mgr_ptr = GetCan2TxMgr();

  vision_rx_mgr_ptr = GetVisionRxMgr();
  vision_tx_mgr_ptr = GetVisionTxMgr();

  rc_rx_mgr_ptr = GetRcRxMgr();
};

static void CommAddReceiver(void) {
  HW_ASSERT(can1_rx_mgr_ptr != nullptr, "can1_rx_mgr_ptr is nullptr",
            can1_rx_mgr_ptr);
  can1_rx_mgr_ptr->addReceiver(GetGimbalChassisComm());
  can1_rx_mgr_ptr->addReceiver(GetMotorYaw());

  HW_ASSERT(can2_rx_mgr_ptr != nullptr, "can2_rx_mgr_ptr is nullptr",
            can2_rx_mgr_ptr);
  can2_rx_mgr_ptr->addReceiver(GetMotorFricLeft());
  can2_rx_mgr_ptr->addReceiver(GetMotorFricRight());
  can2_rx_mgr_ptr->addReceiver(GetMotorFeed());
  can2_rx_mgr_ptr->addReceiver(GetMotorPitch());

  HW_ASSERT(rc_rx_mgr_ptr != nullptr, "rc_rx_mgr_ptr is nullptr",
            rc_rx_mgr_ptr);
  rc_rx_mgr_ptr->addReceiver(GetRemoteControl());

  HW_ASSERT(vision_rx_mgr_ptr != nullptr, "vision_rx_mgr_ptr is nullptr",
            vision_rx_mgr_ptr);
  vision_rx_mgr_ptr->addReceiver(GetVision());
};

static void CommAddTransmitter(void) {
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr",
            can1_tx_mgr_ptr);
  can1_tx_mgr_ptr->addTransmitter(GetGimbalChassisComm());
  can1_tx_mgr_ptr->addTransmitter(GetMotorYaw());

  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr",
            can2_tx_mgr_ptr);
  can2_tx_mgr_ptr->addTransmitter(GetMotorFricLeft());
  can2_tx_mgr_ptr->addTransmitter(GetMotorFricRight());
  can2_tx_mgr_ptr->addTransmitter(GetMotorFeed());
  can2_tx_mgr_ptr->addTransmitter(GetMotorPitch());

  HW_ASSERT(vision_tx_mgr_ptr != nullptr, "vision_tx_mgr_ptr is nullptr",
            vision_tx_mgr_ptr);
  vision_tx_mgr_ptr->addTransmitter(GetVision());
};

void CommHardWareInit(void) {
  // CAN init
  HW_ASSERT(can1_rx_mgr_ptr != nullptr, "can1_rx_mgr_ptr is nullptr",
            can1_rx_mgr_ptr);
  can1_rx_mgr_ptr->filterInit();
  can1_rx_mgr_ptr->startReceive();
  HAL_CAN_Start(&hcan1);

  HW_ASSERT(can2_rx_mgr_ptr != nullptr, "can2_rx_mgr_ptr is nullptr",
            can2_rx_mgr_ptr);
  can2_rx_mgr_ptr->filterInit();
  can2_rx_mgr_ptr->startReceive();
  HAL_CAN_Start(&hcan2);

  // rc DMA init
  HW_ASSERT(rc_rx_mgr_ptr != nullptr, "rc_rx_mgr_ptr is nullptr",
            rc_rx_mgr_ptr);
  rc_rx_mgr_ptr->startReceive();

  // vision DMA init
  HW_ASSERT(vision_rx_mgr_ptr != nullptr, "vision_rx_mgr_ptr is nullptr",
            vision_rx_mgr_ptr);
  vision_rx_mgr_ptr->startReceive();
};

/* Private function definitions
 * ----------------------------------------------*/