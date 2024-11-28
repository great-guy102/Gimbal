/**
 *******************************************************************************
 * @file      : can_rx_mgr.hpp
 * @brief     : CAN 接收管理器
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_BSP_COMM_CAN_RX_MGR_HPP_
#define HW_COMPONENTS_BSP_COMM_CAN_RX_MGR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "stm32_hal.hpp"

/* 开启 CAN 才允许编译 */
#ifdef HAL_CAN_MODULE_ENABLED

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

enum CanRxStatus {
  kCanRxStatusOk = 0,                    ///< 正常
  kCanRxStatusFilterErr = 1 << 0,        ///< 过滤器配置错误
  kCanRxStatusRecvStartFailed = 1 << 1,  ///< 接收开启失败
  kCanRxStatusGetRxMsgErr = 1 << 2,      ///< 获取接收消息错误
};

enum class CanRxType {
  kFifo0 = 0,
  kFifo1 = 1,
};

class CanRxMgr : public RxMgr
{
 public:
  typedef CanRxStatus Status;
  typedef CanRxType RxType;

  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  CanRxMgr(void) = default;
  /**
   * @brief       构造函数
   * @param        hcan: CAN 句柄
   * @param        rx_type: 接收邮箱类型，可选值为：
   *   @arg        RxType::kFifo0: 接收邮箱0
   *   @arg        RxType::kFifo1: 接收邮箱1
   * @retval       None
   * @note        rx_type 需与 STM32CubeMX 中配置的接收中断对应
   */
  CanRxMgr(CAN_HandleTypeDef *hcan, RxType rx_type);
  CanRxMgr(const CanRxMgr &) = default;
  CanRxMgr &operator=(const CanRxMgr &other);
  CanRxMgr(CanRxMgr &&other);
  CanRxMgr &operator=(CanRxMgr &&other);

  virtual ~CanRxMgr(void) = default;

  /* 重载方法 */

  /**
   * @brief       开始 CAN 接收
   * @retval       None
   * @note        只需调用一次
   */
  virtual void startReceive(void) override;

  /**
   * @brief       停止 CAN 接收
   * @retval       None
   * @note        None
   */
  virtual void stopReceive(void) override;

  /* 配置方法 */

  /**
   * @brief       初始化 CAN 接收端，使用默认构造函数后请务必调用此函数
   * @param        hcan: CAN 句柄
   * @param        rx_type: 接收邮箱类型，可选值为：
   *   @arg        RxType::kFifo0: 接收邮箱0
   *   @arg        RxType::kFifo1: 接收邮箱1
   * @retval       None
   * @note        rx_type 需与 STM32CubeMX 中配置的接收中断对应
   */
  void init(CAN_HandleTypeDef *hcan, RxType rx_type);

  /**
   * @brief       初始化 CAN 过滤器
   * @retval       None
   * @note        为接收所有 ID 的过滤器配置，有进一步要求请自行初始化过滤器
   */
  void filterInit(void);

  /* 回调函数 */

  /**
   * @brief       CAN 接收邮箱挂起回调
   * @param        hcan: CAN 句柄
   * @retval       请将次函数放置到对应的 HAL_CAN_RxFifo0MsgPendingCallback 或
   *               HAL_CAN_RxFifo1MsgPendingCallback 函数中
   */
  void rxFifoMsgPendingCallback(CAN_HandleTypeDef *hcan);

  /**
   * @brief       CAN 错误回调
   * @param        hcan: CAN 句柄
   * @retval       None
   * @note        请将此函数放置于所有的 HAL_CAN_ErrorCallback 中，用在总线总线错误
   *              时重启 CAN
   */
  void errorCallback(CAN_HandleTypeDef *hcan);

  /* 数据修改与获取 */

  Status status(void) const { return status_; }

 private:
  /* 硬件相关 */

  CAN_HandleTypeDef *hcan_ = nullptr;
  RxType rx_type_ = RxType::kFifo0;

  /* 接收状态相关 */

  Status status_ = kCanRxStatusOk;
  bool start_receive_ = false;
  uint32_t decode_success_cnt_ = 0;
  uint32_t receive_cnt_ = 0;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace comm
}  // namespace hello_world

#endif /* HAL_CAN_MODULE_ENABLED */

#endif /* HW_COMPONENTS_BSP_COMM_CAN_RX_MGR_HPP_ */
