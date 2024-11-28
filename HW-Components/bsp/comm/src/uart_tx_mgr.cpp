/**
 *******************************************************************************
 * @file      : uart_tx_mgr.cpp
 * @brief     : UART 发送管理器
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-07-10      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  1. 请在 STM32CubeMX 中开启 UART 的 DMA 发送（Normal），同时开启串口全局中断，具
 *  体硬件配置详见 Wiki
 *  2. 由于内部使用了硬件句柄，因此如果计划将实例作为全局变量时（全局变量初始化时对应的
 *  硬件句柄可能会还未初始化完毕），建议采取一下方法：
 *    1）声明指针，后续通过 `new` 的方式进行初始化
 *    2）声明指针，后续通过返回函数（CreateXXXIns）中的静态变量（因为该变量只有在第一
 *    次调用该函数时才会运行初始化程序）进行初始化
 *    3）使用无参构造函数，后续调用 `init` 方法进行初始化
 *    4）使用无参构造函数，后续使用拷贝赋值函数或是移动赋值函数进行初始化
 *  3. H7 系列的 DMA 有工作区域要求，详见 Wiki 上的配置方法
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "uart_tx_mgr.hpp"

/* 开启 UART 与 DMA 才允许编译 */
#if defined(HAL_UART_MODULE_ENABLED) && defined(HAL_DMA_MODULE_ENABLED)

#include <cstring>

#include "allocator.hpp"
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

UartTxMgr::UartTxMgr(UART_HandleTypeDef *huart, size_t buf_len)
    : TxMgr(), huart_(huart), buf_len_(buf_len)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(huart != nullptr, "huart is nullptr");
  HW_ASSERT(buf_len > 0, "buf_len is 0");
#pragma endregion

  tx_buf_ = Allocator<uint8_t>::allocate(buf_len_);
  is_alloc_buf_ = true;
}

UartTxMgr::UartTxMgr(
    UART_HandleTypeDef *huart, uint8_t *tx_buf, size_t buf_len)
    : TxMgr(), huart_(huart), buf_len_(buf_len)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(huart != nullptr, "huart is nullptr");
  HW_ASSERT(buf_len > 0, "buf_len is 0");
#pragma endregion

  tx_buf_ = tx_buf;
  is_alloc_buf_ = false;
}

UartTxMgr &UartTxMgr::operator=(const UartTxMgr &other)
{
  if (this != &other) {
    huart_ = other.huart_;
    buf_len_ = other.buf_len_;
    tx_buf_ = other.tx_buf_;
    is_alloc_buf_ = other.is_alloc_buf_;
    status_ = other.status_;
    start_transmit_ = other.start_transmit_;
  }

  return *this;
}

UartTxMgr::UartTxMgr(UartTxMgr &&other) : TxMgr(std::move(other))
{
  huart_ = other.huart_;
  buf_len_ = other.buf_len_;
  tx_buf_ = other.tx_buf_;
  is_alloc_buf_ = other.is_alloc_buf_;
  status_ = other.status_;
  start_transmit_ = other.start_transmit_;

  other.huart_ = nullptr;
  other.tx_buf_ = nullptr;
  other.is_alloc_buf_ = false;
}

UartTxMgr &UartTxMgr::operator=(UartTxMgr &&other)
{
  if (this != &other) {
    TxMgr::operator=(std::move(other));

    huart_ = other.huart_;
    buf_len_ = other.buf_len_;
    tx_buf_ = other.tx_buf_;
    is_alloc_buf_ = other.is_alloc_buf_;
    status_ = other.status_;
    start_transmit_ = other.start_transmit_;

    other.huart_ = nullptr;
    other.tx_buf_ = nullptr;
    other.is_alloc_buf_ = false;
  }

  return *this;
}

UartTxMgr::~UartTxMgr(void)
{
  if (is_alloc_buf_) {
    Allocator<uint8_t>::deallocate(tx_buf_, buf_len_);
  }
}

void UartTxMgr::startTransmit(void)
{
  start_transmit_ = true;

  /* 避免未发送完再次开启 */
  if (IsBitsSet(HAL_UART_STATE_BUSY_TX,
                HAL_UART_GetState(huart_))) {
    return;
  }

  /* 避免重入 */
  if (IsBitsSet(kUartTxStatusBusy, status_)) {
    return;
  }

  SetBits(kUartTxStatusBusy, status_);

  size_t len = buf_len_;
  uint32_t tx_id = 0;

  /* 编码 */
  uint32_t encode_success_num = encode(len, tx_buf_, tx_id);
  if (encode_success_num) {
    encode_success_cnt_ += encode_success_num;
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(huart_, tx_buf_, len);
    if (status == HAL_OK) {
      transmit_cnt_++;
      setTransmitterFinished(tx_id);
    } else if (status == HAL_ERROR) {
      SetBits(kUartTxStatusTxErr, status_);
    }
  }

  ClearBits(kUartTxStatusBusy, status_);
}

void UartTxMgr::stopTransmit()
{
  start_transmit_ = false;

  HAL_UART_AbortTransmit_IT(huart_);
}

void UartTxMgr::init(UART_HandleTypeDef *huart, size_t buf_len)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(huart != nullptr, "huart is nullptr");
  HW_ASSERT(buf_len > 0, "buf_len is 0");
#pragma endregion

  if (is_alloc_buf_) {
    Allocator<uint8_t>::deallocate(tx_buf_, buf_len_);
  }

  huart_ = huart;
  buf_len_ = buf_len;

  tx_buf_ = Allocator<uint8_t>::allocate(buf_len_);
  is_alloc_buf_ = true;

  status_ = kUartTxStatusOk;
  start_transmit_ = false;
  encode_success_cnt_ = 0;
}

void UartTxMgr::init(UART_HandleTypeDef *huart, uint8_t *tx_buf, size_t buf_len)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(huart != nullptr, "huart is nullptr");
  HW_ASSERT(buf_len > 0, "buf_len is 0");
#pragma endregion

  if (is_alloc_buf_) {
    Allocator<uint8_t>::deallocate(tx_buf_, buf_len_);
  }

  huart_ = huart;
  buf_len_ = buf_len;

  tx_buf_ = tx_buf;
  is_alloc_buf_ = false;

  status_ = kUartTxStatusOk;
  start_transmit_ = false;
  encode_success_cnt_ = 0;
}

void UartTxMgr::txCpltCallback(UART_HandleTypeDef *huart)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(huart != nullptr, "huart is nullptr");
#pragma endregion

  if (huart == huart_ && start_transmit_) {
    startTransmit();
  }
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace comm
}  // namespace hello_world

#endif /* HAL_UART_MODULE_ENABLED && HAL_DMA_MODULE_ENABLED */