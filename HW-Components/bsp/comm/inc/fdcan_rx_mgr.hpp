/**
 *******************************************************************************
 * @file      : fdcan_rx_mgr.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_BSP_COMM_FDCAN_RX_MGR_HPP_
#define HW_COMPONENTS_BSP_COMM_FDCAN_RX_MGR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "stm32_hal.hpp"

/* 开启 FDCAN 才允许编译 */
#ifdef HAL_FDCAN_MODULE_ENABLED

#include "rx_mgr.hpp"
#include "system.hpp"

namespace hello_world
{
namespace comm
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START

enum FdCanRxStatus {
  kFdCanRxStatusOk = 0,                     ///< 正常
  kFdCanRxStatusFilterErr = 1 << 0,         ///< 过滤器配置错误
  kFdCanRxStatusItLinesConfigErr = 1 << 1,  ///< 中断线配置错误
  kFdCanRxStatusRecvStartFailed = 1 << 2,   ///< 接收开启失败
  kFdCanRxStatusGetRxMsgErr = 1 << 3,       ///< 获取接收消息错误
};

enum class FdCanRxType {
  kFifo0 = 0,
  kFifo1 = 1,
};

enum class FdCanRxItLine {
  k0 = 0,
  k1 = 1,
};
class FdCanRxMgr : public RxMgr
{
 public:
  typedef FdCanRxStatus Status;
  typedef FdCanRxType RxType;
  typedef FdCanRxItLine ItLine;

  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  FdCanRxMgr(void) = default;
  /**
   * @brief       构造函数
   * @param        hfdcan: FDCAN 句柄
   * @param        rx_type: 接收邮箱类型，可选值为：
   *   @arg        RxType::kFifo0: FIFO0
   *   @arg        RxType::kFifo1: FIFO1
   * @retval       None
   * @note        None
   */
  FdCanRxMgr(FDCAN_HandleTypeDef *hfdcan, RxType rx_type);
  FdCanRxMgr(const FdCanRxMgr &) = default;
  FdCanRxMgr &operator=(const FdCanRxMgr &other);
  FdCanRxMgr(FdCanRxMgr &&other);
  FdCanRxMgr &operator=(FdCanRxMgr &&other);

  virtual ~FdCanRxMgr(void) = default;

  /* 重载方法 */

  /**
   * @brief       开始 FDCAN 接收
   * @retval       None
   * @note        只需调用一次
   */
  virtual void startReceive(void) override;

  /**
   * @brief       停止 FDCAN 接收
   * @retval       None
   * @note        None
   */
  virtual void stopReceive(void) override;

  /* 配置方法 */

  /**
   * @brief       初始化 FDCAN 接收管理器，使用默认构造函数后请务必调用此函数
   * @param        hfdcan: FDCAN 句柄
   * @param        rx_type: 接收邮箱类型，可选值为：
   *   @arg        RxType::kFifo0: FIFO0
   *   @arg        RxType::kFifo1: FIFO1
   * @retval       None
   * @note        None
   */
  void init(FDCAN_HandleTypeDef *hfdcan, RxType rx_type);

  /**
   * @brief       初始化 FDCAN 过滤器
   * @retval       None
   * @note        为接收所有 ID 的过滤器配置，有进一步要求请自行初始化过滤器
   */
  void filterInit(void);

  /**
   * @brief       配置 FDCAN 中断线
   * @param        it_line: 中断线序号，可选值为：
   *   @arg        ItLine::k0: 中断线0
   *   @arg        ItLine::k1: 中断线1
   * @retval       None
   * @note        必须在 startReceive() 之前调用，同时需与 STM32CubeMX 中配置的中断
   *              对应
   */
  void configInterruptLines(ItLine it_line);

  /* 回调函数 */

  /**
   * @brief       FDCAN 接收邮箱挂起回调
   * @param        hfdcan: FDCAN 句柄
   * @retval       请将次函数放置到对应的 HAL_FDCAN_RxFifo0Callback 或
   * HAL_FDCAN_RxFifo1Callback 函数中
   */
  void rxFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo_its);

  /**
   * @brief       FDCAN 错误状态回调
   * @param        hfdcan: FDCAN 句柄
   * @param        error_status_its: 错误状态中断标志
   * @retval       请将次函数放置到 HAL_FDCAN_ErrorStatusCallback 函数中，用于在
   *               Bus-off 时重置 FDCAN
   */
  void errorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t error_status_its);

  /* 数据修改与获取 */

  Status status(void) const { return status_; }

 private:
  /* 硬件相关 */

  FDCAN_HandleTypeDef *hfdcan_ = nullptr;
  RxType rx_type_ = RxType::kFifo0;

  /* 接收状态相关 */

  Status status_ = kFdCanRxStatusOk;
  bool start_receive_ = false;
  uint32_t decode_success_cnt_ = 0;
  uint32_t receive_cnt_ = 0;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace comm
}  // namespace hello_world

#endif /* HAL_FDCAN_MODULE_ENABLED */

#endif /* HW_COMPONENTS_BSP_COMM_FDCAN_RX_MGR_HPP_ */
