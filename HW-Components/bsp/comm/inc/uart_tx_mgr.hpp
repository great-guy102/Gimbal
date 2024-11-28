/**
 *******************************************************************************
 * @file      : uart_tx_mgr.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_BSP_COMM_UART_TX_MGR_HPP_
#define HW_COMPONENTS_BSP_COMM_UART_TX_MGR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "stm32_hal.hpp"

/* 开启 UART 与 DMA 才允许编译 */
#if defined(HAL_UART_MODULE_ENABLED) && defined(HAL_DMA_MODULE_ENABLED)

#include "system.hpp"
#include "tx_mgr.hpp"

namespace hello_world
{
namespace comm
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START

enum UartTxStatus {
  kUartTxStatusOk = 0,          ///< 正常
  kUartTxStatusBusy = 1 << 0,   ///< 忙
  kUartTxStatusTxErr = 1 << 1,  ///< 发送错误
};
class UartTxMgr : public TxMgr
{
 public:
  typedef UartTxStatus Status;

  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  UartTxMgr(void) = default;
  /**
   * @brief       构造函数
   * @param        huart: UART 句柄
   * @param        buf_len: 发送缓冲区长度
   * @retval       None
   * @note        None
   */
  UartTxMgr(UART_HandleTypeDef *huart, size_t buf_len);
  /**
   * @brief       构造函数
   * @param        huart: UART 句柄
   * @param        tx_buf: 发送缓冲区，使用期间不可释放
   * @param        buf_len: 发送缓存长度
   * @retval       None
   * @note        对于 DMA 由工作区域要求的 H7 系列，需要使用此构造函数
   */
  UartTxMgr(UART_HandleTypeDef *huart, uint8_t *tx_buf, size_t buf_len);
  UartTxMgr(const UartTxMgr &) = default;
  UartTxMgr &operator=(const UartTxMgr &other);
  UartTxMgr(UartTxMgr &&other);
  UartTxMgr &operator=(UartTxMgr &&other);

  virtual ~UartTxMgr(void);

  /* 重载方法 */

  /**
   * @brief       开始 UART 发送
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
   * @brief       停止 UART 发送
   * @retval       None
   * @note        None
   */
  virtual void stopTransmit(void) override;

  /* 配置方法 */

  /**
   * @brief       初始化，使用默认构造函数后请务必调用此函数
   * @param       huart: UART 句柄
   * @param       buf_len: 发送缓冲区长度
   * @retval       None
   * @note        None
   */
  void init(UART_HandleTypeDef *huart, size_t buf_len);

  /**
   * @brief       初始化，使用默认构造函数后请务必调用此函数
   * @param       huart: UART 句柄
   * @param       tx_buf: 发送缓冲区，使用期间不可释放
   * @param       buf_len: 发送缓存长度
   * @retval       None
   * @note        对于 DMA 由工作区域要求的 H7 系列，需要使用此初始化函数
   */
  void init(UART_HandleTypeDef *huart, uint8_t *tx_buf, size_t buf_len);

  /* 回调函数 */

  /**
   * @brief       UART 发送完成回调
   * @param       huart: UART 句柄
   * @retval       None
   * @note        请将次函数放置到对应的 HAL_UART_TxCpltCallback 函数中，用于在完成
   *              一条报文的 DMA 发送后继续发送下一条报文
   */
  void txCpltCallback(UART_HandleTypeDef *huart);

  /* 数据修改与获取 */

  Status status(void) const { return status_; }

 private:
  /* 硬件相关 */

  UART_HandleTypeDef *huart_ = nullptr;

  /* 发送状态相关 */

  Status status_ = kUartTxStatusOk;
  bool start_transmit_ = false;
  uint32_t encode_success_cnt_ = 0;
  uint32_t transmit_cnt_ = 0;

  /* 发送数据相关 */

  size_t buf_len_ = 0;
  uint8_t *tx_buf_ = nullptr;  ///< 发送缓冲区
  bool is_alloc_buf_ = false;  ///< 缓冲区是否为内部分配
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace comm
}  // namespace hello_world

#endif /* HAL_UART_MODULE_ENABLED && HAL_DMA_MODULE_ENABLED */

#endif /* HW_COMPONENTS_BSP_COMM_UART_TX_MGR_HPP_ */
