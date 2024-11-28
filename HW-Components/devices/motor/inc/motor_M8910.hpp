/**
 *******************************************************************************
 * @file      : motor_M8910.hpp
 * @brief     : 自研电机（苏机）类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-25      Caikunzhen      1. 未测试版本
 *  V1.0.0      2024-07-11      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  1. 请先查看 motor_base.hpp 中的注意事项
 *  2. 电机 ID 范围为 1~8，其中 1~4 为同一条发送报文，5~8 为同一条发送报文
 *  3. 可使用的输入类型为 InputType::kRaw、InputType::kTorq 和 InputType::kCurr
 *  4. 可额外获得的数据为电机功率 power
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_MOTOR_MOTOR_M8910_HPP_
#define HW_COMPONENTS_DEVICES_MOTOR_MOTOR_M8910_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor_base.hpp"
#include "system.hpp"

namespace hello_world
{
namespace motor
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START

class M8910 : public Motor
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  M8910(void) = default;
  /**
   * @brief       M8910 初始化
   * @param        id: 电机 ID，1~8
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  explicit M8910(
      uint8_t id, const OptionalParams& opt = OptionalParams());
  M8910(const M8910&) = default;
  M8910& operator=(const M8910& other);
  M8910(M8910&& other);
  M8910& operator=(M8910&& other);

  virtual ~M8910(void) = default;

  /* 重载方法 */

  /**
   * @brief       将电调发回的 CAN 报文进行解包
   * @param        len: 报文长度
   * @param        data: 电调发回的 CAN 报文
   * @retval       是否解包成功
   * @note        请前判断 rx_id 是否符合再进行解码
   */
  virtual bool decode(size_t len, const uint8_t* data) override;

  /**
   * @brief       将要发给电调的期望输值编码为对应的 CAN 报文
   * @param        len: 传入缓冲区长度（必须为 8），返回报文长度
   * @param        data: 将要发出的 CAN 报文
   * @retval       是否编码成功
   * @note        请前判断 tx_id 是否符合再进行解码
   */
  virtual bool encode(size_t& len, uint8_t* data) override;

  /* 配置方法 */

  /**
   * @brief       M8910 初始化，使用默认构造函数后请务必调用此函数
   * @param        id: 电机 ID，1~8
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  void init(uint8_t id, const OptionalParams& opt = OptionalParams());

  /* 数据修改与获取 */

  float power(void) const { return power_; }

  /**
   * @brief       根据 id 获取对应的发送 ID
   * @param        id: 电机 ID，范围 1~8
   * @retval       对应的发送 ID
   * @note        None
   */
  static uint32_t GetTxId(uint8_t id)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(1 <= id && id <= 8, "Error id: %d", id);
#pragma endregion

    if (id < 4) {
      return kTx1_4_ + id;
    } else {
      return kTx5_8_ + id;
    }
  }

  /**
   * @brief       根据 id 获取对应的接收 ID
   * @param        id: 电机 ID，范围 1~8
   * @retval       对应的接收 ID
   * @note        None
   */
  static uint32_t GetRxId(uint8_t id)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(1 <= id && id <= 8, "Error id: %d", id);
#pragma endregion

    return kRx0_ + id;
  }

 private:
  /* 电机状态 */

  float power_ = 0;  ///* 电机功率

  static constexpr uint32_t kTx1_4_ = 0xFF;
  static constexpr uint32_t kTx5_8_ = 0x100;
  static constexpr uint32_t kRx0_ = 0x100;
  static constexpr float power_rat_ = 1000.0f / 0x8000;
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace motor
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_MOTOR_MOTOR_M8910_HPP_ */
