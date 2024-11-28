/**
 *******************************************************************************
 * @file      : motor_DM_J8006.hpp
 * @brief     : 达妙 DM-J8006 电机类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2024-05-01      Caikunzhen      1. 未测试版本
 *  V1.0.0      2024-07-11      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  1. 请先查看 motor_base.hpp 中的注意事项
 *  2. 电机 ID 范围为 1~10，每个电机 ID 对应一个发送报文
 *  3. 可使用的输入类型为 InputType::kRaw、InputType::kTorq、InputType::kCurr 和
 *  InputType::kCmd。其中要注意在 InputType::kRaw 中输入值恒为正，且不具有大小比较关
 *  系，详见对应的电机说明手册。对于 InputType::kCmd，建议的使用方法如下：
 *  ```cpp
 *  /* 记录原始输入类型 * /
 *  InputType original_input_type = motor.get_input_type();
 *  motor.set_input_type(InputType::kCmd);
 *  motor.setInput(DM_J8006::Cmd::kCmdEnable); // 使能电机
 *  motor.setInput(DM_J8006::Cmd::kCmdDisable); // 失能电机
 *  motor.setInput(DM_J8006::Cmd::kCmdClearErr); // 清除错误
 *  /* 恢复原始输入类型 * /
 *  motor.set_input_type(original_input_type);
 *  ```
 *  4. 电机的命令优先级高于设置的电机输入，只有当电机没有需要发送的命令时，才会发送设置
 *  的电机输入
 *  5. 使用电机前需要使用上位机对电机进行配置，其中 Master ID 为 ID+0x10，CAN ID 为
 *  ID+0x00，PMAX 为 3.141593，VMAX 为 21，TMAX 为 21
 *  6. 该电机上电时处于失能状态，需要通过发送指令使能电机，否则电机不会工作，当电机初始
 *  化中 auto_enable 为 true 时，当电机反馈表明电机失能时会自动使能电机，同时如果需要
 *  电机处于失能状态，则需要持续输入失能命令。若 auto_enable 为 false 时，需要手动使能
 *  电机。同时，当电机处于错误状态时，会自动清除错误并重启电机。上电后会一直发送使能指令，
 *  直到电机反馈使能成功
 *  7. 电机为一发一收的通信方式，不给电机发送指令时电机不会反馈数据
 *  8. 可额外获得的数据为转子温度 rotor_temp、MOS管温度 mos_temp 和状态码
 *  status_code
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_MOTOR_MOTOR_DM_J8006_HPP_
#define HW_COMPONENTS_DEVICES_MOTOR_MOTOR_DM_J8006_HPP_

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

class DM_J8006 : public Motor
{
 public:
  enum class StatusCode {
    kMotorDisabled = 0x0,  ///* 电机失能
    kMotorEnabled = 0x1,   ///* 电机使能
    kOverVolt = 0x8,       ///* 过压
    kUnderVolt = 0x9,      ///* 欠压
    kOverCurr = 0xA,       ///* 过流
    kMosOverTemp = 0xB,    ///* MOS过温
    kCoilOverTemp = 0xC,   ///* 电机线圈过温
    kCommLoss = 0xD,       ///* 通信丢失
    kOverload = 0xE,       ///* 过载
  };

  enum Cmd {
    kCmdNone = 0,           ///* 无指令
    kCmdClearErr = 1 << 0,  ///* 清除错误
    kCmdDisable = 1 << 1,   ///* 电机失能
    kCmdEnable = 1 << 2,    ///* 电机使能
  };

  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  DM_J8006(void) = default;
  /**
   * @brief       DM_J8006 初始化
   * @param        id: 电机 ID，1~10
   * @param        opt: 电机可选配置参数
   * @param        auto_enable: 电机自动使能
   * @retval       None
   * @note        None
   */
  explicit DM_J8006(uint8_t id,
                    const OptionalParams& opt = OptionalParams(),
                    bool auto_enable = true);
  DM_J8006(const DM_J8006&) = default;
  DM_J8006& operator=(const DM_J8006& other);
  DM_J8006(DM_J8006&& other);
  DM_J8006& operator=(DM_J8006&& other);

  virtual ~DM_J8006(void) = default;

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
   * @brief       将原始报文内容转换为输出端力矩
   * @param        raw: 原始报文数值
   * @retval       原始报文对应的输出端力矩值，单位：N·m
   * @note        报文的对应的物理意义需要与设定的相符，设定对应情可以通过 motor_info
   *              方法获取电机信息结构体，查看其中的 raw_mapping_type 变量
   */
  virtual float raw2Torq(float raw) const override;

  /**
   * @brief       将输出端力矩转换为原始报文内容
   * @param        torq: 输出端力矩值，单位：N·m
   * @retval       输出端力矩值对应的原始报文
   * @note        报文的对应的物理意义需要与设定的相符，设定对应情可以通过 motor_info
   *              方法获取电机信息结构体，查看其中的 raw_mapping_type 变量
   */
  virtual float torq2Raw(float torq) const override;

  /**
   * @brief       将原始报文内容转换为转子电流
   * @param        raw: 原始报文数值
   * @retval       原始报文对应的转子电流值，单位：A
   * @note        报文的对应的物理意义需要与设定的相符，设定对应情可以通过 motor_info
   *              方法获取电机信息结构体，查看其中的 raw_mapping_type 变量
   */
  virtual float raw2Curr(float raw) const override;

  /**
   * @brief       将转子电流转换为原始报文内容
   * @param        curr: 转子电流值，单位：A
   * @retval       转子电流值对应的原始报文
   * @note        报文的对应的物理意义需要与设定的相符，设定对应情可以通过 motor_info
   *              方法获取电机信息结构体，查看其中的 raw_mapping_type 变量
   */
  virtual float curr2Raw(float curr) const override;

  /**
   * @brief       设定发给电调的期望值
   * @param        input: 发给电调的期望值，当输入类型为 InputType::kCmd 时，可选值
   *               为：
   *   @arg        DM_J8006::Cmd::kCmdEnable: 使能电机
   *   @arg        DM_J8006::Cmd::kCmdDisable: 失能电机
   *   @arg        DM_J8006::Cmd::kCmdClearErr: 清除错误
   * @retval       设置状态，可能的返回值有：
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
   *   @arg        InputType::kTorq: 输出端力矩输入
   *   @arg        InputType::kCurr: 转子端电流输入
   *   @arg        InputType::kCmd: 命令输入
   * @retval       设置状态，可能的返回值有：
   *   @arg        Status::kOk: 设置成功
   *   @arg        Status::kInputTypeError: 输入类型错误
   * @note        None
   */
  virtual Status set_input_type(InputType input_type) override;

  /* 配置方法 */

  /**
   * @brief       DM_J8006 初始化，使用默认构造函数后请务必调用此函数
   * @param        id: 电机 ID，1~10
   * @param        opt: 电机可选配置参数
   * @param        auto_enable: 电机自动使能
   * @retval       None
   * @note        None
   */
  void init(uint8_t id, const OptionalParams& opt = OptionalParams(),
            bool auto_enable = true);

  /* 数据修改与获取 */

  uint8_t rotor_temp(void) const { return rotor_temp_; }

  uint8_t mos_temp(void) const { return mos_temp_; }

  StatusCode status_code(void) const { return status_code_; }

  /**
   * @brief       根据 id 获取对应的发送 ID
   * @param        id: 电机 ID，范围 1~10
   * @retval       对应的发送 ID
   * @note        None
   */
  static uint32_t GetTxId(uint8_t id)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(1 <= id && id <= 10, "Error id: %d", id);
#pragma endregion

    return kTx0_ + id;
  }

  /**
   * @brief       根据 id 获取对应的接收 ID
   * @param        id: 电机 ID，范围 1~10
   * @retval       对应的接收 ID
   * @note        None
   */
  static uint32_t GetRxId(uint8_t id)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(1 <= id && id <= 10, "Error id: %d", id);
#pragma endregion

    return kRx0_ + id;
  }

 private:
  /* 电机状态 */

  uint8_t rotor_temp_ = 0;     ///* 电机转子温度
  uint8_t mos_temp_ = 0;       ///* 电机MOS管温度
  bool is_connected_ = false;  ///* 电机是否连接
  bool is_enabled_ = false;    ///* 电机是否使能

  StatusCode status_code_ = StatusCode::kMotorDisabled;  ///* 状态码

  bool auto_enable_ = true;               ///* 电机自动使能
  Cmd wait_to_handle_cmd_ = kCmdDisable;  ///* 待处理的指令

  static constexpr uint32_t kTx0_ = 0x00;
  static constexpr uint32_t kRx0_ = 0x10;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace motor
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_MOTOR_MOTOR_DM_J8006_HPP_ */
