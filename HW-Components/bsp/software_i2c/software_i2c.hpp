/**
 *******************************************************************************
 * @file      : software_i2c.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_BSP_SOFTWARE_I2C_SOFTWARE_I2C_HPP_
#define HW_COMPONENTS_BSP_SOFTWARE_I2C_SOFTWARE_I2C_HPP_

/* Includes ------------------------------------------------------------------*/
#include "stm32_hal.hpp"

/* 开启 GPIO 才允许编译 */
#if defined(HAL_GPIO_MODULE_ENABLED)

#include <cstddef>
#include <cstdint>

#include "allocator.hpp"
#include "tick.hpp"

namespace hello_world
{
namespace software_i2c
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class SoftwareI2c : public MemMgr
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  SoftwareI2c(void) = default;
  /**
   * @brief       软件 I2C 初始化
   * @param       scl_port: SCL 引脚所在的 GPIO 端口
   * @param       scl_pin: SCL 引脚编号，可选值为：
   *   @arg       gpio_pin_x: x=0, 1, 2, ..., 15
   * @param       sda_port: SDA 引脚所在的 GPIO 端口
   * @param       sda_pin: SDA 引脚编号，可选值为：
   *   @arg       gpio_pin_x: x=0, 1, 2, ..., 15
   * @retval       none
   * @note        none
   */
  SoftwareI2c(GPIO_TypeDef* scl_port, uint16_t scl_pin,
              GPIO_TypeDef* sda_port, uint16_t sda_pin);
  SoftwareI2c(const SoftwareI2c&) = default;
  SoftwareI2c& operator=(const SoftwareI2c& other) = default;
  SoftwareI2c(SoftwareI2c&& other);
  SoftwareI2c& operator=(SoftwareI2c&& other);

  ~SoftwareI2c(void) = default;

  /* 配置方法 */

  /**
   * @brief       软件 I2C 初始化，使用默认构造函数后请务必调用此函数
   * @param       scl_port: SCL 引脚所在的 GPIO 端口
   * @param       scl_pin: SCL 引脚编号，可选值为：
   *   @arg       gpio_pin_x: x=0, 1, 2, ..., 15
   * @param       sda_port: SDA 引脚所在的 GPIO 端口
   * @param       sda_pin: SDA 引脚编号，可选值为：
   *   @arg       gpio_pin_x: x=0, 1, 2, ..., 15
   * @retval       none
   * @note        none
   */
  void init(GPIO_TypeDef* scl_port, uint16_t scl_pin,
            GPIO_TypeDef* sda_port, uint16_t sda_pin);

  /* 功能性方法 */

  /**
   * @brief       发送数据
   * @param       addr: 从机地址，无需左移一位
   * @param       size: 发送数据的长度
   * @param       data: 发送的数据
   * @retval       是否发送成功
   * @note        None
   */
  bool transmit(uint8_t addr, size_t size, const uint8_t* data) const
  {
    start();
    writeByte(addr << 1);
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

  /**
   * @brief       读取数据
   * @param       addr: 从机地址，无需左移一位
   * @param       size: 读取数据的长度
   * @param       data: 读取到的数据
   * @retval       是否读取成功
   * @note        None
   */
  bool receive(uint8_t addr, size_t size, uint8_t* data) const
  {
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

  /**
   * @brief       写入数据到从机的指定地址
   * @param       addr: 从机地址，无需左移一位
   * @param       mem_addr: 从机内存地址
   * @param       size: 写入数据的长度
   * @param       data: 写入的数据
   * @retval       是否写入成功
   * @note        None
   */
  bool memWrite(
      uint8_t addr, uint8_t mem_addr, size_t size, const uint8_t* data) const;

  /**
   * @brief       读取从机的指定地址的数据
   * @param       addr: 从机地址，无需左移一位
   * @param       mem_addr: 从机内存地址
   * @param       size: 读取数据的长度
   * @param       data: 读取到的数据
   * @retval       是否读取成功
   * @note        None
   */
  bool memRead(
      uint8_t addr, uint8_t mem_addr, size_t size, uint8_t* data) const;

  void start(void) const
  {
    sdaOut();
    sdaH();
    sclH();
    tick::DelayUs(2);
    sdaL();
    tick::DelayUs(2);
    sclL();
    tick::DelayUs(1);
  }

  void stop(void) const
  {
    sdaOut();
    sdaL();
    sclL();
    tick::DelayUs(2);
    sclH();
    tick::DelayUs(2);
    sdaH();
    tick::DelayUs(2);
  }

  void writeByte(uint8_t data) const
  {
    sdaOut();
    for (uint8_t i = 0; i < 8; i++) {
      if (data & 0x80) {
        sdaH();
      } else {
        sdaL();
      }
      tick::DelayUs(1);
      sclH();
      tick::DelayUs(1);
      sclL();
      tick::DelayUs(1);
      data <<= 1;
    }
  }

  /**
   * @brief       读取一个字节
   * @param       is_ack: 是否发送应答信号
   * @retval       读取到的数据
   * @note        None
   */
  uint8_t readByte(bool is_ack) const
  {
    uint8_t data = 0;
    sdaH();
    sdaIn();
    for (uint8_t i = 0; i < 8; i++) {
      tick::DelayUs(1);
      sclH();
      data <<= 1;
      if (HAL_GPIO_ReadPin(sda_port_, sda_pin_) == GPIO_PIN_SET) {
        data |= 0x01;
      }
      tick::DelayUs(1);
      sclL();
      tick::DelayUs(1);
    }
    if (is_ack) {
      ack();
    } else {
      nAck();
    }
    return data;
  }

  void nAck(void) const
  {
    sdaOut();
    sdaH();
    tick::DelayUs(1);
    sclH();
    tick::DelayUs(1);
    sclL();
    tick::DelayUs(1);
  }

  void ack(void) const
  {
    sdaOut();
    sdaL();
    tick::DelayUs(1);
    sclH();
    tick::DelayUs(1);
    sclL();
    tick::DelayUs(1);
  }

  /**
   * @brief       等待应答信号
   * @retval       是否收到应答信号
   * @note        None
   */
  bool waitAck(void) const
  {
    sdaH();
    sdaIn();
    tick::DelayUs(1);
    sclH();
    tick::DelayUs(1);
    bool is_ack = HAL_GPIO_ReadPin(sda_port_, sda_pin_) == GPIO_PIN_RESET;
    sclL();
    tick::DelayUs(1);
    return is_ack;
  }

 private:
  /* 功能性方法 */

  void sclH(void) const
  {
    HAL_GPIO_WritePin(scl_port_, scl_pin_, GPIO_PIN_SET);
  }

  void sclL(void) const
  {
    HAL_GPIO_WritePin(scl_port_, scl_pin_, GPIO_PIN_RESET);
  }

  void sdaH(void) const
  {
    HAL_GPIO_WritePin(sda_port_, sda_pin_, GPIO_PIN_SET);
  }

  void sdaL(void) const
  {
    HAL_GPIO_WritePin(sda_port_, sda_pin_, GPIO_PIN_RESET);
  }

  /**
   * @brief       将 SDA 置为输入模式
   * @retval       None
   * @note        None
   */
  void sdaIn(void) const
  {
    sda_port_->MODER &= ~(0x3 << (sda_mode_reg_idx_ * 2));
    sda_port_->MODER |= 0x0 << (sda_mode_reg_idx_ * 2);
  }

  /**
   * @brief       将 SDA 置为输出模式
   * @retval       None
   * @note        None
   */
  void sdaOut(void) const
  {
    sda_port_->MODER &= ~(0x3 << (sda_mode_reg_idx_ * 2));
    sda_port_->MODER |= 0x1 << (sda_mode_reg_idx_ * 2);
  }

  GPIO_TypeDef* scl_port_ = nullptr;
  uint16_t scl_pin_ = GPIO_PIN_0;
  GPIO_TypeDef* sda_port_ = nullptr;
  uint16_t sda_pin_ = GPIO_PIN_0;
  uint8_t sda_mode_reg_idx_ = 0;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace software_i2c
}  // namespace hello_world

#endif /* HAL_GPIO_MODULE_ENABLED */

#endif /* HW_COMPONENTS_BSP_SOFTWARE_I2C_SOFTWARE_I2C_HPP_ */
