/**
 *******************************************************************************
 * @file      : motor_DM_J8006.cpp
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
/* Includes ------------------------------------------------------------------*/
#include "motor_DM_J8006.hpp"

#include <string.h>

#include "assert.hpp"

namespace hello_world
{
namespace motor
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static const MotorBaseInfo kDM_J8006MotorBaseInfo{
    .raw_input_lim = kInvalidValue,
    .torq_input_lim = 20.0f,
    .curr_input_lim = 18.0f,
    .torq_const = 1.23f,
    .redu_rat = 1.0f,  ///* DM-J8006 为输出端编码，且减速器（6:1）难以拆卸
    .angle_rat = kInvalidValue,
    .vel_rat = kInvalidValue,
    .curr_rat = kInvalidValue,
    .torq_rat = kInvalidValue,
    .cross_0_value = static_cast<uint16_t>(kInvalidValue),
    .raw_mapping_type = RawMappingType::kTorq,
};

static const float kMaxAngle = PI;
static const uint8_t kAngleBits = 16;
static const float kMaxVel = 21.0f;
static const uint8_t kVelBits = 12;
static const float kMaxTorq = 21.0f;
static const uint8_t kTorqBits = 12;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

HW_OPTIMIZE_O2_START

/**
 * @brief       根据给定的范围和位数，将 uint 转换为 float
 * @param        x_uint: 待转换的 uint
 * @param        x_max: 给定范围的最大值
 * @param        x_min: 给定范围的最小值
 * @param        bits: 整形的位数
 * @retval       转换后的 float 变量
 * @note        该函数仅供 DM_J8006 电机数据处理用
 */
static inline float Uint2Float(
    uint16_t x_int, float x_max, float x_min, uint8_t bits);

/**
 * @brief       根据给定的范围和位数，将 float 转换为 uint
 * @param        x_float: 待转换的 float
 * @param        x_max: 给定范围的最大值
 * @param        x_min: 给定范围的最小值
 * @param        bits: 整形的位数
 * @retval       转换后的 uint 变量
 * @note        该函数仅供 DM_J8006 电机数据处理用
 */
static inline uint16_t Float2Uint(
    float x_float, float x_max, float x_min, uint8_t bits);
/* Exported function definitions ---------------------------------------------*/

DM_J8006::DM_J8006(uint8_t id, const OptionalParams& opt,
                   bool auto_enable) : Motor(opt.offline_tick_thres)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(1 <= id && id <= 10, "Error id: %d", id);
  HW_ASSERT(opt.dir == kDirFwd || opt.dir == kDirRev,
            "Error dir: %d", opt.dir);
  HW_ASSERT(opt.angle_range == AngleRange::k0To2Pi ||
                opt.angle_range == AngleRange::kNegInfToPosInf ||
                opt.angle_range == AngleRange::kNegPiToPosPi,
            "Error angle range: %d", opt.angle_range);
  HW_ASSERT(opt.input_type == InputType::kRaw ||
                opt.input_type == InputType::kTorq ||
                opt.input_type == InputType::kCurr,
            "Error input type: %d", opt.input_type);
  HW_ASSERT(opt.ex_redu_rat > 0,
            "Error external reduction ration: %f", opt.ex_redu_rat);
  HW_ASSERT(opt.max_torq_input_lim > 0,
            "Error max torque input limit: %f", opt.max_torq_input_lim);
  HW_ASSERT(opt.max_curr_input_lim > 0,
            "Error max current input limit: %f", opt.max_curr_input_lim);
#pragma endregion

  motor_info_ = kDM_J8006MotorBaseInfo;

  /* 根据 ID 设定报文 ID */
  motor_info_.tx_id = kTx0_ + id;
  motor_info_.rx_id = kRx0_ + id;
  motor_info_.id = id;

  motor_info_.dir = opt.dir;
  motor_info_.angle_range = opt.angle_range;
  motor_info_.input_type = opt.input_type;
  motor_info_.angle_offset = opt.angle_offset;

  /* 转子端力矩限制 */
  float rotor_torq_lim = motor_info_.torq_input_lim / motor_info_.redu_rat;
  if (opt.remove_build_in_reducer) {
    motor_info_.redu_rat = opt.ex_redu_rat;
  } else {
    motor_info_.redu_rat *= opt.ex_redu_rat;
  }

  /* 计算输入限制 */
  float max_torq_input = rotor_torq_lim * motor_info_.redu_rat;
  if (opt.max_torq_input_lim != std::numeric_limits<float>::max()) {
    max_torq_input = std::min(max_torq_input, opt.max_torq_input_lim);
  }
  float max_curr_input = motor_info_.curr_input_lim;
  if (opt.max_curr_input_lim != std::numeric_limits<float>::max()) {
    max_curr_input = std::min(max_curr_input, opt.max_curr_input_lim);
  }

  max_torq_input = std::min(max_torq_input, kMaxTorq);
  max_torq_input = std::min(max_torq_input, curr2Torq(max_curr_input));
  motor_info_.raw_input_lim = kInvalidValue;
  motor_info_.torq_input_lim = max_torq_input;
  motor_info_.curr_input_lim = torq2Curr(max_torq_input);

  auto_enable_ = auto_enable;
}

DM_J8006& DM_J8006::operator=(const DM_J8006& other)
{
  if (this != &other) {
    Motor::operator=(other);

    rotor_temp_ = other.rotor_temp_;
    mos_temp_ = other.mos_temp_;
    is_connected_ = other.is_connected_;
    is_enabled_ = other.is_enabled_;

    status_code_ = other.status_code_;

    auto_enable_ = other.auto_enable_;
    wait_to_handle_cmd_ = other.wait_to_handle_cmd_;
  }

  return *this;
}

DM_J8006::DM_J8006(DM_J8006&& other) : Motor(std::move(other))
{
  if (this != &other) {
    rotor_temp_ = other.rotor_temp_;
    mos_temp_ = other.mos_temp_;
    is_connected_ = other.is_connected_;
    is_enabled_ = other.is_enabled_;

    status_code_ = other.status_code_;

    auto_enable_ = other.auto_enable_;
    wait_to_handle_cmd_ = other.wait_to_handle_cmd_;
  }
}

DM_J8006& DM_J8006::operator=(DM_J8006&& other)
{
  if (this != &other) {
    Motor::operator=(std::move(other));

    rotor_temp_ = other.rotor_temp_;
    mos_temp_ = other.mos_temp_;
    is_connected_ = other.is_connected_;
    is_enabled_ = other.is_enabled_;

    status_code_ = other.status_code_;

    auto_enable_ = other.auto_enable_;
    wait_to_handle_cmd_ = other.wait_to_handle_cmd_;
  }

  return *this;
}

bool DM_J8006::decode(size_t len, const uint8_t* data)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(data, "data is nullptr");
#pragma endregion

  if (len != 8) {
    decode_fail_cnt_++;
    return false;
  }

  uint16_t raw_angle = static_cast<uint16_t>((data[1] << 8) | data[2]);
  float angle = Uint2Float(raw_angle, kMaxAngle, -kMaxAngle, kAngleBits);
  float last_angle = Uint2Float(
      last_raw_angle_, kMaxAngle, -kMaxAngle, kAngleBits);

  /* 判断是否旋转了一圈 */
  float round = round_;
  float delta_angle = angle - last_angle;
  if (fabsf(delta_angle) > 2 * PI * kCross0ValueThres) {
    delta_angle < 0 ? round++ : round--;

    /* 避免圈数溢出 */
    if (motor_info_.angle_range != AngleRange::kNegInfToPosInf) {
      round = fmodf(round, motor_info_.redu_rat);
    }
  }

  float actual_ang =
      static_cast<float>(motor_info_.dir) * (angle + round * 2 * PI) /
      motor_info_.redu_rat;
  float raw = static_cast<uint16_t>(((data[4] & 0x0F) << 8) | data[5]);

  /* 统一对电机状态赋值 */
  round_ = round;
  actual_angle_ = normAngle(actual_ang);
  angle_ = normAngle(actual_ang - motor_info_.angle_offset);
  vel_raw_ = static_cast<float>(motor_info_.dir) *
             Uint2Float(
                 static_cast<int16_t>((data[3] << 4) | (data[3] >> 4)),
                 kMaxVel, -kMaxVel, kVelBits) /
             motor_info_.redu_rat;
  vel_ = calcVel();
  torq_ = static_cast<float>(motor_info_.dir) * raw2Torq(raw);
  curr_ = static_cast<float>(motor_info_.dir) * raw2Curr(raw);
  mos_temp_ = data[6];
  rotor_temp_ = data[7];
  status_code_ = static_cast<StatusCode>(data[0] >> 4);

  if (status_code_ == StatusCode::kMotorEnabled) {
    is_enabled_ = true;
  } else if (status_code_ == StatusCode::kMotorDisabled) {
    is_enabled_ = false;
  } else {
    SetBits(kCmdClearErr, wait_to_handle_cmd_);
    SetBits(kCmdDisable, wait_to_handle_cmd_);
    SetBits(kCmdEnable, wait_to_handle_cmd_);
  }

  last_raw_angle_ = raw_angle;

  is_connected_ = true;

  oc_.update();

  decode_success_cnt_++;
  is_update_ = true;
  if (update_cb_) {
    update_cb_();
  }
  return true;
}

bool DM_J8006::encode(size_t& len, uint8_t* data)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(data, "data is nullptr");
#pragma endregion

  if (len != 8) {
    encode_fail_cnt_++;
    return false;
  }

  if (wait_to_handle_cmd_ == kCmdNone) {
    if (!is_enabled_ && auto_enable_) {
      SetBits(kCmdEnable, wait_to_handle_cmd_);
    } else {
      uint16_t input = curr2Raw(
          static_cast<float>(motor_info_.dir) * curr_input_);
      input = hello_world::Bound(
          input, static_cast<uint16_t>(0),
          static_cast<uint16_t>((0x01 << kTorqBits) - 1));
      memset(data, 0x00, sizeof(uint8_t) * 7);
      data[6] = (input >> 8) & 0x0F;
      data[7] = input & 0xFF;
    }
  }

  /* 处理指令 */
  if (IsBitsSet(kCmdClearErr, wait_to_handle_cmd_)) {
    memset(data, 0xFF, sizeof(uint8_t) * 7);
    data[7] = 0xFB;
    if (is_connected_) {
      ClearBits(kCmdClearErr, wait_to_handle_cmd_);
    }
  } else if (IsBitsSet(kCmdDisable, wait_to_handle_cmd_)) {
    memset(data, 0xFF, sizeof(uint8_t) * 7);
    data[7] = 0xFD;
    if (is_connected_) {
      ClearBits(kCmdDisable, wait_to_handle_cmd_);
    }
  } else if (IsBitsSet(kCmdEnable, wait_to_handle_cmd_)) {
    memset(data, 0xFF, sizeof(uint8_t) * 7);
    data[7] = 0xFC;
    if (is_connected_) {
      ClearBits(kCmdEnable, wait_to_handle_cmd_);
    }
  }

  encode_success_cnt_++;
  return true;
}

float DM_J8006::raw2Torq(float raw) const
{
  return Uint2Float(raw, kMaxTorq, -kMaxTorq, kTorqBits) *
         motor_info_.redu_rat;
}

float DM_J8006::torq2Raw(float torq) const
{
  return Float2Uint(
      torq / motor_info_.redu_rat, kMaxTorq, -kMaxTorq, kTorqBits);
}

float DM_J8006::raw2Curr(float raw) const
{
  return Uint2Float(raw, kMaxTorq, -kMaxTorq, kTorqBits) /
         motor_info_.torq_const;
}

float DM_J8006::curr2Raw(float curr) const
{
  return Float2Uint(
      curr * motor_info_.torq_const, kMaxTorq, -kMaxTorq, kTorqBits);
}

Status DM_J8006::setInput(float input)
{
  float torq_input = 0;
  switch (motor_info_.input_type) {
    case InputType::kRaw:
      torq_input = raw2Torq(input);
      torq_input_ = hello_world::Bound(
          input, motor_info_.torq_input_lim, -motor_info_.torq_input_lim);
      curr_input_ = torq2Curr(torq_input_);
      raw_input_ = torq2Raw(torq_input_);
      if (fabsf(torq_input_) < fabsf(torq_input)) {
        return Status::kInputValueOverflow;
      } else {
        return Status::kOk;
      }
    case InputType::kTorq:
      torq_input_ = hello_world::Bound(
          input, motor_info_.torq_input_lim, -motor_info_.torq_input_lim);
      raw_input_ = torq2Raw(torq_input_);
      curr_input_ = torq2Curr(torq_input_);
      if (fabsf(torq_input_) < fabsf(input)) {
        return Status::kInputValueOverflow;
      } else {
        return Status::kOk;
      }
    case InputType::kCurr:
      curr_input_ = hello_world::Bound(
          input, motor_info_.curr_input_lim, -motor_info_.curr_input_lim);
      raw_input_ = curr2Raw(curr_input_);
      torq_input_ = curr2Torq(curr_input_);
      if (fabsf(curr_input_) < fabsf(input)) {
        return Status::kInputValueOverflow;
      } else {
        return Status::kOk;
      }
    case InputType::kCmd:
      if (input == kCmdEnable || input == kCmdDisable ||
          input == kCmdClearErr) {
        SetBits(Cmd(input), wait_to_handle_cmd_);
        return Status::kOk;
      } else {
        return Status::kInputValueOverflow;
      }
      break;
    default:
      return Status::kInputTypeError;
  }
}

Status DM_J8006::set_input_type(InputType input_type)
{
  if (input_type == InputType::kRaw || input_type == InputType::kTorq ||
      input_type == InputType::kCurr || input_type == InputType::kCmd) {
    motor_info_.input_type = input_type;
    return Status::kOk;
  } else {
    return Status::kInputTypeError;
  }
}

void DM_J8006::init(uint8_t id, const OptionalParams& opt, bool auto_enable)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(1 <= id && id <= 10, "Error id: %d", id);
  HW_ASSERT(opt.dir == kDirFwd || opt.dir == kDirRev,
            "Error dir: %d", opt.dir);
  HW_ASSERT(opt.angle_range == AngleRange::k0To2Pi ||
                opt.angle_range == AngleRange::kNegInfToPosInf ||
                opt.angle_range == AngleRange::kNegPiToPosPi,
            "Error angle range: %d", opt.angle_range);
  HW_ASSERT(opt.input_type == InputType::kRaw ||
                opt.input_type == InputType::kTorq ||
                opt.input_type == InputType::kCurr,
            "Error input type: %d", opt.input_type);
  HW_ASSERT(opt.ex_redu_rat > 0,
            "Error external reduction ration: %f", opt.ex_redu_rat);
  HW_ASSERT(opt.max_torq_input_lim > 0,
            "Error max torque input limit: %f", opt.max_torq_input_lim);
  HW_ASSERT(opt.max_curr_input_lim > 0,
            "Error max current input limit: %f", opt.max_curr_input_lim);
#pragma endregion

  motor_info_ = kDM_J8006MotorBaseInfo;

  /* 根据 ID 设定报文 ID */
  motor_info_.tx_id = kTx0_ + id;
  motor_info_.rx_id = kRx0_ + id;
  motor_info_.id = id;

  motor_info_.dir = opt.dir;
  motor_info_.angle_range = opt.angle_range;
  motor_info_.input_type = opt.input_type;
  motor_info_.angle_offset = opt.angle_offset;

  /* 转子端力矩限制 */
  float rotor_torq_lim = motor_info_.torq_input_lim / motor_info_.redu_rat;
  if (opt.remove_build_in_reducer) {
    motor_info_.redu_rat = opt.ex_redu_rat;
  } else {
    motor_info_.redu_rat *= opt.ex_redu_rat;
  }

  /* 计算输入限制 */
  float max_torq_input = rotor_torq_lim * motor_info_.redu_rat;
  if (opt.max_torq_input_lim != std::numeric_limits<float>::max()) {
    max_torq_input = std::min(max_torq_input, opt.max_torq_input_lim);
  }
  float max_curr_input = motor_info_.curr_input_lim;
  if (opt.max_curr_input_lim != std::numeric_limits<float>::max()) {
    max_curr_input = std::min(max_curr_input, opt.max_curr_input_lim);
  }

  max_torq_input = std::min(max_torq_input, kMaxTorq);
  max_torq_input = std::min(max_torq_input, curr2Torq(max_curr_input));
  motor_info_.raw_input_lim = kInvalidValue;
  motor_info_.torq_input_lim = max_torq_input;
  motor_info_.curr_input_lim = torq2Curr(max_torq_input);

  auto_enable_ = auto_enable;

  angle_ = 0.0f;
  vel_ = 0.0f;
  vel_raw_ = 0.0f;
  torq_ = 0.0f;
  curr_ = 0.0f;
  round_ = 0.0f;
  actual_angle_ = 0.0f;

  is_update_ = false;
  update_cb_ = nullptr;
  oc_.init(opt.offline_tick_thres);

  raw_input_ = 0;
  torq_input_ = 0;
  curr_input_ = 0;

  td_ptr_ = nullptr;

  decode_success_cnt_ = 0;
  decode_fail_cnt_ = 0;
  encode_success_cnt_ = 0;
  encode_fail_cnt_ = 0;
  transmit_success_cnt_ = 0;

  rotor_temp_ = 0;
  mos_temp_ = 0;
  is_connected_ = false;
  is_enabled_ = false;
  status_code_ = StatusCode::kMotorDisabled;

  wait_to_handle_cmd_ = kCmdDisable;
}
/* Private function definitions ----------------------------------------------*/

static inline float Uint2Float(
    uint16_t x_int, float x_max, float x_min, uint8_t bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static inline uint16_t Float2Uint(
    float x_float, float x_max, float x_min, uint8_t bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return (uint16_t)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

HW_OPTIMIZE_O2_END
}  // namespace motor
}  // namespace hello_world
