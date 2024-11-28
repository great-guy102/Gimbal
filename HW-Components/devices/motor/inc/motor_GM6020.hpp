/**
 *******************************************************************************
 * @file      : motor_GM6020.hpp
 * @brief     : 大疆 GM6020 电机类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-25      Caikunzhen      1. 未测试版本
 *  V1.0.0      2023-12-04      Caikunzhen      1. 完成测试
 *  V1.1.0      2024-07-11      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  1. 请先查看 motor_base.hpp 中的注意事项
 *  2. 电机 ID 范围为 1~7，其中 1~4 为同一条发送报文，5~7 为同一条发送报文
 *  3. 由于电机为电压控制因此只能使用 InputType::kRaw 输入方式
 *  4. raw2x 与 x2raw 方法只适用于反馈报文中的转换关系，无法用于控制报文
 *  5. 可额外获得的数据为电机温度 temp
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_MOTOR_MOTOR_GM6020_HPP_
#define HW_COMPONENTS_DEVICES_MOTOR_MOTOR_GM6020_HPP_

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

class GM6020 : public Motor
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  GM6020(void) = default;
  /**
   * @brief       GM6020 初始化
   * @param        id: 电机 ID，1~7
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  explicit GM6020(
      uint8_t id, const OptionalParams& opt = OptionalParams());
  GM6020(const GM6020&) = default;
  GM6020& operator=(const GM6020& other);
  GM6020(GM6020&& other);
  GM6020& operator=(GM6020&& other);

  virtual ~GM6020(void) = default;

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

  /**
   * @brief       设定发给电调的期望值
   * @param        input: 发给电调的期望值
   * @retval       设定状态，可能的返回值有：
   *   @arg        Status::kOk: 设定成功
   *   @arg        Status::kInputTypeError: 输入类型错误
   *   @arg        Status::kInputValueOverflow: 设定值超出范围
   * @note        1. 期望值的物理意义与电机当前的输入类型有关，可使用 get_input_type
   *              方法查看
   *              2. 设定的期望值会自动被限制到允许的范围内，当前实际的设定值可以通过
   *              getInput 方法查看
   */
  virtual Status setInput(float input) override;

  /**
   * @brief       设置点击的输入类型
   * @param        input_type: 期望输入类型，可选值为：
   *   @arg        InputType::kRaw: 原始报文输入
   * @retval       设置状态，可能的返回值有：
   *   @arg        Status::kOk: 设置成功
   *   @arg        Status::kInputTypeError: 输入类型错误
   * @note        None
   */
  virtual Status set_input_type(InputType input_type) override;

  /* 配置方法 */

  /**
   * @brief       GM6020 初始化，使用默认构造函数后请务必调用此函数
   * @param        id: 电机 ID，1~7
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  void init(uint8_t id, const OptionalParams& opt = OptionalParams());

  /* 数据修改与获取 */

  uint8_t temp(void) const { return temp_; }

  /**
   * @brief       根据 id 获取对应的发送 ID
   * @param        id: 电机 ID，范围 1~7
   * @retval       对应的发送 ID
   * @note        None
   */
  static uint32_t GetTxId(uint8_t id)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(1 <= id && id <= 7, "Error id: %d", id);
#pragma endregion

    if (id < 4) {
      return kTx1_4_ + id;
    } else {
      return kTx5_7_ + id;
    }
  }

  /**
   * @brief       根据 id 获取对应的接收 ID
   * @param        id: 电机 ID，范围 1~7
   * @retval       对应的接收 ID
   * @note        None
   */
  static uint32_t GetRxId(uint8_t id)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(1 <= id && id <= 7, "Error id: %d", id);
#pragma endregion

    return kRx0_ + id;
  }

 private:
  /* 电机状态 */

  uint8_t temp_ = 0;  ///* 电机温度

  static constexpr uint32_t kTx1_4_ = 0x1FF;
  static constexpr uint32_t kTx5_7_ = 0x2FF;
  static constexpr uint32_t kRx0_ = 0x204;
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace motor
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_MOTOR_MOTOR_M3508_HPP_ */
