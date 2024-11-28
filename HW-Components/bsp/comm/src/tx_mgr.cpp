/**
 *******************************************************************************
 * @file      : tx_mgr.cpp
 * @brief     : 通信发送管理器基类
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-07-10      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  不建议用户使用及继承该类，而是使用其派生类
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "tx_mgr.hpp"

#include <cstring>

#include "assert.hpp"
#include "stm32_hal.hpp"

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
TxMgr &TxMgr::operator=(const TxMgr &other)
{
  if (this != &other) {
    transmitter_ptrs_tx_id_list_ = other.transmitter_ptrs_tx_id_list_;
  }

  return *this;
}

TxMgr::TxMgr(TxMgr &&other)
    : transmitter_ptrs_tx_id_list_(
          std::move(other.transmitter_ptrs_tx_id_list_))
{
}

TxMgr &TxMgr::operator=(TxMgr &&other)
{
  if (this != &other) {
    transmitter_ptrs_tx_id_list_ =
        std::move(other.transmitter_ptrs_tx_id_list_);
  }

  return *this;
}

void TxMgr::addTransmitter(Transmitter *new_transmitter_ptr)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(new_transmitter_ptr != nullptr, "new_transmitter_ptr is nullptr");
#pragma endregion

  /* 遍历查看 ID 是否已存在 */
  for (auto &it : transmitter_ptrs_tx_id_list_) {
    if (it.tx_id() == new_transmitter_ptr->txId()) {
      /* 查看发送端是否已存在，当不存在时再添加 */
      for (auto &transmitter_ptr : it.transmitter_ptrs()) {
        if (transmitter_ptr == new_transmitter_ptr) {
          return;
        }
      }
      it.transmitter_ptrs().push_back(new_transmitter_ptr);

      return;
    }
  }

  /* ID 不存在则创建新的 ID 并插入新发送端 */
  transmitter_ptrs_tx_id_list_.emplace_back(
      new_transmitter_ptr->txId(), new_transmitter_ptr);
}

void TxMgr::setTransmitterNeedToTransmit(const Transmitter *transmitter_ptr)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(transmitter_ptr != nullptr, "transmitter_ptr is nullptr");
#pragma endregion

  for (auto &it : transmitter_ptrs_tx_id_list_) {
    if (it.tx_id() == transmitter_ptr->txId()) {
      it.need_to_transmit() = true;
      return;
    }
  }
}

bool TxMgr::rmTransmittersFromTxId(uint32_t tx_id)
{
  for (auto it = transmitter_ptrs_tx_id_list_.begin();
       it != transmitter_ptrs_tx_id_list_.end(); ++it) {
    if (it->tx_id() == tx_id) {
      transmitter_ptrs_tx_id_list_.erase(it);
      return true;
    }
  }

  return false;
}

void TxMgr::clearTransmitter(void)
{
  transmitter_ptrs_tx_id_list_.clear();
}

size_t TxMgr::getRemainMsgNum(void)
{
  size_t cnt = 0;
  for (auto &it : transmitter_ptrs_tx_id_list_) {
    if (it.need_to_transmit() == true) {
      cnt++;
    }
  }

  return cnt;
}

void TxMgr::getRemainIds(uint32_t *tx_ids, size_t &len)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(tx_ids != nullptr, "tx_ids is nullptr");
  HW_ASSERT(len > 0, "len is less than 0");
#pragma endregion

  size_t cnt = 0;
  memset(tx_ids, 0, len);
  for (auto &it : transmitter_ptrs_tx_id_list_) {
    if (it.need_to_transmit() == true) {
      if (cnt < len) {
        tx_ids[cnt] = it.tx_id();
      }

      cnt++;
    }
  }

  len = cnt;
}

size_t TxMgr::encode(size_t &len, uint8_t *tx_buf, uint32_t &tx_id)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(tx_buf != nullptr, "data is nullptr");
  HW_ASSERT(len > 0, "len is less than 0");
#pragma endregion

  size_t cnt = 0;      // 编码成功的发送端数量
  size_t max_len = 0;  // 最大数据长度

  /* 尝试编码直至有报文编码完全通过时停止或遍历完所有报文 */
  for (auto &it : transmitter_ptrs_tx_id_list_) {
    if (it.need_to_transmit() == false) {
      continue;
    }

    auto &transmitter_ptr_list = it.transmitter_ptrs();
    cnt = 0;
    max_len = 0;

    memset(tx_buf, 0, len);
    /* 对所选报文的发送端依次进行编译 */
    for (auto &transmitter_ptr : transmitter_ptr_list) {
      size_t tmp_buf_len = len;
      if (transmitter_ptr->encode(tmp_buf_len, tx_buf)) {
        max_len = std::max(max_len, tmp_buf_len);
        cnt++;
      } else {
        cnt = 0;
        it.need_to_transmit() = false;  // 取消发送
        break;
      }
    }

    /* 完成编译，返回信息 */
    if (cnt != 0) {
      tx_id = it.tx_id();
      len = max_len;
      return cnt;
    }
  }

  return cnt;
}

void TxMgr::setTransmitterFinished(uint32_t tx_id)
{
  for (auto &it : transmitter_ptrs_tx_id_list_) {
    if (it.tx_id() == tx_id) {
      /* 设置报文发送完成并调用对应 ID 的发送端的发送完成回调 */
      it.need_to_transmit() = false;
      for (auto &transmitter_ptr : it.transmitter_ptrs()) {
        transmitter_ptr->txSuccessCb();
      }

      return;
    }
  }
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace comm
}  // namespace hello_world