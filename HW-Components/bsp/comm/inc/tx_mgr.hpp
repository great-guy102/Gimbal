/**
 *******************************************************************************
 * @file      : tx_mgr.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_BSP_COMM_TX_MGR_HPP_
#define HW_COMPONENTS_BSP_COMM_TX_MGR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "list.hpp"
#include "system.hpp"
#include "transmitter.hpp"

namespace hello_world
{
namespace comm
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START
/**
 * @brief       通信发送管理器基类
 * @note        不建议用户使用及继承该类，而是使用其派生类
 */
class TxMgr : public MemMgr
{
 public:
  TxMgr(void) = default;
  TxMgr(const TxMgr &) = default;
  TxMgr &operator=(const TxMgr &other);
  TxMgr(TxMgr &&other);
  TxMgr &operator=(TxMgr &&other);

  virtual ~TxMgr(void) = default;

  /* 功能性方法 */

  /**
   * @brief       开启发送
   * @retval       None
   * @note        方法用于在所有发送端添加完毕后开启发送，开启后相同 ID 的发送端将被依
   *              次调用 encode 方法对同一条报文进行编码，完成后再将报文通过外设发出，
   *              为适配各个发送端可能有不同的发送频率或是交替发送的特殊需求，因此每次
   *              要发送数据时都需要先调用 setTransmitterNeedToTransmit 方法再调用
   *              发送开启方法，同时对于某个 ID 对应的报文，只有所有发送端编码均成功，
   *              发送管理器才会认为该报文处于可发送状态
   */
  virtual void startTransmit(void) = 0;

  virtual void stopTransmit(void) = 0;

  /**
   * @brief       添加发送端
   * @param        new_transmitter_ptr: 发送端指针，在运行期间不能被释放，建议为静态
   *               变量或全局变量
   * @retval       None
   * @note        已有的发送端不会被重复添加
   */
  void addTransmitter(Transmitter *new_transmitter_ptr);

  /**
   * @brief       设置需要发送的发送端
   * @param        transmitter_ptr: 发送端指针
   * @retval       None
   * @note        每次调用后最多只会发送一次（编码有可能失败），需要再次发送需要再次调
   *              用，同时仅根据 transmitter_ptr 的 txId 将对应的报文设置为需要发送，
   *              不会检查 transmitter_ptr 是否在发送端列表中
   */
  void setTransmitterNeedToTransmit(const Transmitter *transmitter_ptr);

  /**
   * @brief       根据发送 ID 移除发送端
   * @param        tx_id: 发送 ID
   * @retval       移除成功返回 true，否则返回 false
   * @note        不建议在发送过程中移除发送端
   */
  bool rmTransmittersFromTxId(uint32_t tx_id);

  /**
   * @brief       清除发送端
   * @retval       None
   * @note        不建议在发送过程中清除发送端
   */
  void clearTransmitter(void);

  /* 数据修改与获取 */

  /**
   * @brief       获取剩余发送数据条数
   * @param        None
   * @retval       剩余发送数据条数，不一定是发送端数量
   * @note        建议在调试期间在每次循环添加发送端前调用该方法，如果发现待发送报文数
   *              量不为零，说明上一次循环添加的报文未能及时全部发出，此时应考虑相应的
   *              解决方法
   */
  size_t getRemainMsgNum(void);

  /**
   * @brief       获取剩余发送 ID
   * @param        tx_ids: 用于存放剩余发送 ID 的缓冲区，多余的部分被置为零，若缓冲区
   *               大小小于剩余发送 ID 数量，则只返回缓冲区大小的数据
   * @param        len: 传入缓冲区的大小，返回剩余发送 ID 的数量
   * @retval       None
   * @note        None
   */
  void getRemainIds(uint32_t *tx_ids, size_t &len);

 protected:
  class TransmitterPtrsTxId : public MemMgr
  {
   public:
    typedef tools::list<Transmitter *> TransmitterPtrs;

    /**
     * @brief       构造函数
     * @param        tx_id: 发送 ID
     * @param        transmitter_ptr: 发送端指针
     * @retval       None
     * @note        None
     */
    TransmitterPtrsTxId(uint32_t tx_id, Transmitter *transmitter_ptr)
        : tx_id_(tx_id)
    {
      if (transmitter_ptr != nullptr) {
        transmitter_ptrs_.push_back(transmitter_ptr);
      }
    }
    TransmitterPtrsTxId(const TransmitterPtrsTxId &) = default;
    TransmitterPtrsTxId &operator=(const TransmitterPtrsTxId &) = delete;
    TransmitterPtrsTxId(TransmitterPtrsTxId &&) = delete;
    TransmitterPtrsTxId &operator=(TransmitterPtrsTxId &&) = delete;

    virtual ~TransmitterPtrsTxId(void) = default;

    /* 功能性方法 */

    /**
     * @brief       判断是否为同一对象
     * @param        other: 另一个对象
     * @retval       是返回 true，否则返回 false
     * @note        None
     */
    bool operator==(const TransmitterPtrsTxId &other) const
    {
      if (&other == this) {
        return true;
      }

      return false;
    }

    /* 数据修改与获取 */

    TransmitterPtrs &transmitter_ptrs(void) { return transmitter_ptrs_; }

    uint32_t tx_id(void) const { return tx_id_; }

    bool need_to_transmit(void) const { return need_to_transmit_; }

    bool &need_to_transmit(void) { return need_to_transmit_; }

   private:
    TransmitterPtrs transmitter_ptrs_;
    uint32_t tx_id_;
    bool need_to_transmit_ = false;
  };

  typedef tools::list<TransmitterPtrsTxId> TransmitterPtrsTxIdList;

  /* 功能性方法 */

  /**
   * @brief       编码
   * @param        len: 传入缓冲区的大小，返回数据长度（最大）
   * @param        tx_buf: 用于编码的缓冲区，注意即使编码失败也会有部分数据写入
   * @param        tx_id: 当前编译的发送 ID
   * @retval       编码成功成功时报文对应的发送端数量，否则返回 0
   * @note        当某条报文对应的所有编码器有一个编码失败时会取消改条报文的编码，并转
   *              而编码下一条报文
   */
  size_t encode(size_t &len, uint8_t *tx_buf, uint32_t &tx_id);

  /**
   * @brief       设置发送端完成
   * @param        tx_id: 发送 ID
   * @retval       None
   * @note        None
   */
  void setTransmitterFinished(uint32_t tx_id);

  TransmitterPtrsTxIdList transmitter_ptrs_tx_id_list_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace comm
}  // namespace hello_world

#endif /* HW_COMPONENTS_BSP_COMM_TX_MGR_HPP_ */
