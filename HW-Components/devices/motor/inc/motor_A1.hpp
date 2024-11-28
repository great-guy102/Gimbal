/**
 *******************************************************************************
 * @file      : motor_A1.hpp
 * @brief     : 宇树 A1 电机类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-25      Caikunzhen      1. 未测试版本
 *  V1.0.0      2024-07-11      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  1. 请先查看 motor_base.hpp 中的注意事项
 *  2. 电机 ID 范围为 1~6，其中 1~3 为同一条发送报文，4~6 为同一条发送报文
 *  3. 可使用的输入类型为 InputType::kRaw、InputType::kTorq 和 InputType::kCurr
 *  4. 由于电机原始通信协议为 RS485，因此需要结合队内制作的 CAN 转 RS485 模块使用（与
 *  宇树 GO-M8010 电机不通用），该模块具有两路 RS485 通信接口，一路上上位机配置电机 ID
 *  0~2 对应电机 ID 1~3，另一路对应电机 ID 4~6
 *  5. 可额外获得的数据为电机温度 temp
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_MOTOR_MOTOR_A1_HPP_
#define HW_COMPONENTS_DEVICES_MOTOR_MOTOR_A1_HPP_

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

class A1 : public Motor
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  A1(void) = default;
  /**
   * @brief       A1 初始化
   * @param        id: 电机 ID，1~6
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  explicit A1(
      uint8_t id, const OptionalParams& opt = OptionalParams());
  A1(const A1&) = default;
  A1& operator=(const A1& other);
  A1(A1&& other);
  A1& operator=(A1&& other);

  virtual ~A1(void) = default;

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
   * @brief       A1 初始化，使用默认构造函数后请务必调用此函数
   * @param        id: 电机 ID，1~6
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  void init(uint8_t id, const OptionalParams& opt = OptionalParams());

  /* 数据修改与获取 */

  uint8_t temp(void) const { return temp_; }

  /**
   * @brief       根据 id 获取对应的发送 ID
   * @param        id: 电机 ID，范围 1~6
   * @retval       对应的发送 ID
   * @note        None
   */
  static uint32_t GetTxId(uint8_t id)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(1 <= id && id <= 6, "Error id: %d", id);
#pragma endregion

    if (id < 4) {
      return kTx1_3_ + id;
    } else {
      return kTx4_6_ + id;
    }
  }

  /**
   * @brief       根据 id 获取对应的接收 ID
   * @param        id: 电机 ID，范围 1~6
   * @retval       对应的接收 ID
   * @note        None
   */
  static uint32_t GetRxId(uint8_t id)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(1 <= id && id <= 6, "Error id: %d", id);
#pragma endregion

    return kRx0_ + id;
  }

 private:
  /* 电机状态 */

  uint8_t temp_ = 0;  ///* 电机温度

  static constexpr uint32_t kTx1_3_ = 0x400;
  static constexpr uint32_t kTx4_6_ = 0x3FF;
  static constexpr uint32_t kRx0_ = 0x400;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace motor
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_MOTOR_MOTOR_A1_HPP_ */
