/**
 *******************************************************************************
 * @file      : software_i2c.cpp
 * @brief     : 用于实现软件 I2C 的驱动
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2024-06-21      Caikunzhen      1. 待测试
 *******************************************************************************
 * @attention : 请将 SCL 与 SDA 引脚配置为推挽输出模式，同时 SDA 配置为上拉
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "software_i2c.hpp"

#if defined(HAL_GPIO_MODULE_ENABLED)

#include "assert.hpp"
#include "tick.hpp"

namespace hello_world
{
namespace software_i2c
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
/* Private function definitions ----------------------------------------------*/

SoftwareI2c::SoftwareI2c(GPIO_TypeDef* scl_port, uint16_t scl_pin,
                         GPIO_TypeDef* sda_port, uint16_t sda_pin)
    : scl_port_(scl_port),
      scl_pin_(scl_pin),
      sda_port_(sda_port),
      sda_pin_(sda_pin)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(scl_port != nullptr, "SCL port is nullptr");
  HW_ASSERT(sda_port != nullptr, "SDA port is nullptr");
  HW_ASSERT(IS_GPIO_PIN(scl_pin), "SCL pin is invalid");
  HW_ASSERT(IS_GPIO_PIN(sda_pin), "SDA pin is invalid");
#pragma endregion

  while (sda_pin >>= 1) {
    sda_mode_reg_idx_++;
  }
}

SoftwareI2c::SoftwareI2c(SoftwareI2c&& other)
    : scl_port_(other.scl_port_),
      scl_pin_(other.scl_pin_),
      sda_port_(other.sda_port_),
      sda_pin_(other.sda_pin_),
      sda_mode_reg_idx_(other.sda_mode_reg_idx_)
{
  other.scl_port_ = nullptr;
  other.sda_port_ = nullptr;
}

SoftwareI2c& SoftwareI2c::operator=(SoftwareI2c&& other)
{
  if (this != &other) {
    scl_port_ = other.scl_port_;
    scl_pin_ = other.scl_pin_;
    sda_port_ = other.sda_port_;
    sda_pin_ = other.sda_pin_;
    sda_mode_reg_idx_ = other.sda_mode_reg_idx_;
    other.scl_port_ = nullptr;
    other.sda_port_ = nullptr;
  }
  return *this;
}

void SoftwareI2c::init(GPIO_TypeDef* scl_port, uint16_t scl_pin,
                       GPIO_TypeDef* sda_port, uint16_t sda_pin)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(scl_port != nullptr, "SCL port is nullptr");
  HW_ASSERT(sda_port != nullptr, "SDA port is nullptr");
  HW_ASSERT(IS_GPIO_PIN(scl_pin), "SCL pin is invalid");
  HW_ASSERT(IS_GPIO_PIN(sda_pin), "SDA pin is invalid");
#pragma endregion

  scl_port_ = scl_port;
  scl_pin_ = scl_pin;
  sda_port_ = sda_port;
  sda_pin_ = sda_pin;
  sda_mode_reg_idx_ = 0;
  while (sda_pin >>= 1) {
    sda_mode_reg_idx_++;
  }
}

bool SoftwareI2c::memWrite(
    uint8_t addr, uint8_t mem_addr, size_t size, const uint8_t* data) const
{
  start();
  writeByte(addr << 1);
  if (!waitAck()) {
    stop();
    return false;
  }
  writeByte(mem_addr);
  if (!waitAck()) {
    stop();
    return false;
  }
  for (size_t i = 0; i < size; i++) {
    writeByte(data[i]);
    if (!waitAck()) {
      stop();
      return false;
    }
  }
  stop();
  return true;
}

bool SoftwareI2c::memRead(
    uint8_t addr, uint8_t mem_addr, size_t size, uint8_t* data) const
{
  start();
  writeByte(addr << 1);
  if (!waitAck()) {
    stop();
    return false;
  }
  writeByte(mem_addr);
  if (!waitAck()) {
    stop();
    return false;
  }
  start();
  writeByte((addr << 1) | 0x01);
  if (!waitAck()) {
    stop();
    return false;
  }
  for (size_t i = 0; i < size; i++) {
    data[i] = readByte(i != size - 1);  // 最后一个字节不发送应答信号
  }
  stop();
  return true;
}
}  // namespace software_i2c
}  // namespace hello_world

#endif /* HAL_GPIO_MODULE_ENABLED */
