/**
 *******************************************************************************
 * @file      : super_cap.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2024-07-26      ZhouShichan        1. 初步完成，未测试
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "super_cap.hpp"

#include "assert.hpp"
#include "base.hpp"
namespace hello_world
{

namespace cap
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
HW_OPTIMIZE_O2_START

bool SuperCap::decode(size_t len, const uint8_t *data)
{
  bool res = false;
  if (version_ == Version::kVer2021) {
    res = decodeVer2021(len, data);
  }
  if (version_ == Version::kVer2024) {
    res = decodeVer2024(len, data);
  }
  if (res) {
    decode_success_cnt_++;
    is_updated_ = true;
    oc_.update();
    if (update_cb_ != nullptr) {
      update_cb_();
    }
  } else {
    decode_fail_cnt_++;
  }

  return res;
};

bool SuperCap::encode(size_t &len, uint8_t *data)
{
  bool res = false;
  if (version_ == Version::kVer2021) {
    res = encodeVer2021(len, data);
  }
  if (version_ == Version::kVer2024) {
    res = encodeVer2024(len, data);
  }
  if (res) {
    encode_success_cnt_++;
  } else {
    encode_fail_cnt_++;
  }
  return res;
};

bool SuperCap::setConfig(const Config &cfg)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(cfg.max_charge_volt > 0.0,
            "cfg.max_charge_volt must be greater than 0");
  HW_ASSERT(cfg.min_valid_volt > 0.0,
            "cfg.min_valid_volt must be greater than 0");
  HW_ASSERT(cfg.min_valid_volt <= cfg.max_charge_volt,
            " cfg.min_valid_volt must be less than cfg.max_charge_volt");
  HW_ASSERT(cfg.auto_disable_power >= 0 && cfg.auto_disable_power < 100,
            "cfg.auto_disable_power must be in [0, 100)");
  HW_ASSERT(cfg.min_enable_power >= 0 && cfg.min_enable_power < 100,
            "cfg.min_enable_power must be in [0, 100)");
  HW_ASSERT(cfg.min_enable_power > cfg.auto_disable_power,
            "cfg.min_enable_power must be higher than cfg.auto_disable_power");
#pragma endregion

  if (cfg.max_charge_volt <= 0 || cfg.min_valid_volt <= 0 ||
      cfg.min_valid_volt > cfg.max_charge_volt ||
      cfg.auto_disable_power < 0 || cfg.auto_disable_power >= 1 ||
      cfg.min_enable_power < 0 || cfg.min_enable_power >= 1 ||
      cfg.min_enable_power >= cfg.auto_disable_power) {
    return false;
  }

  cfg_ = cfg;
  cfg_.pwr_filter_beta = Bound(cfg_.pwr_filter_beta, 0.0f, 1.0f);
  return true;
};

bool SuperCap::decodeVer2021(size_t len, const uint8_t *data)
{
  if (len < 3) {
    return false;
  }

  volt_ = (float)((int16_t)(data[0] << 8 | data[1])) / 1000.0f;
  // 计算电压平方
  float volt_sq = volt_ * volt_;
  float min_volt_sq = cfg_.min_valid_volt * cfg_.min_valid_volt;
  float max_volt_sq = cfg_.max_charge_volt * cfg_.max_charge_volt;

  // 计算剩余功率
  float remaining_pwr = (volt_sq - min_volt_sq) / (max_volt_sq - min_volt_sq) *
                        100.0f;
  remaining_pwr = Bound(remaining_pwr, 0.0f, 100.0f);
  remaining_pwr_ = cfg_.pwr_filter_beta * remaining_pwr_ +
                   (1.0f - cfg_.pwr_filter_beta) * remaining_pwr;

  pwr_src_ = PwrSrc(data[3]);

  return true;
};

bool SuperCap::encodeVer2021(size_t &len, uint8_t *data)
{
  if (len < 3) {
    return false;
  }
  uint16_t tmp_uint16 = 0u;
  tmp_uint16 = (uint16_t)rfr_chas_pwr_buffer_;
  data[0] = (uint8_t)tmp_uint16;

  tmp_uint16 = (uint16_t)enable_flag_;
  data[1] = (uint8_t)tmp_uint16;

  tmp_uint16 = (uint16_t)rfr_chas_pwr_limit_;
  data[2] = (uint8_t)tmp_uint16;
  return true;
};

bool SuperCap::decodeVer2024(size_t len, const uint8_t *data)
{
  if (len < 6) {
    return false;
  }

  uint16_t tmp_uint16 = 0u;

  tmp_uint16 = (uint16_t)(data[0] << 8 | data[1]);
  float remaining_pwr = Bound((float)tmp_uint16 / 100.0f, 0.0f, 100.0f);
  remaining_pwr_ = cfg_.pwr_filter_beta * remaining_pwr_ +
                   (1.0f - cfg_.pwr_filter_beta) * remaining_pwr;

  pwr_src_ = PwrSrc(data[2]);

  tmp_uint16 = (uint16_t)(data[3] << 8 | data[4]);
  out_pwr_ = (float)tmp_uint16 / 100.0f;

  tmp_uint16 = (uint16_t)(data[5] << 8 | data[6]);
  volt_ = (float)tmp_uint16 / 100.0f;

  return true;
};

bool SuperCap::encodeVer2024(size_t &len, uint8_t *data)
{
  if (len < 8) {
    return false;
  }

  uint16_t tmp_uint16 = 0u;

  tmp_uint16 = (uint16_t)rfr_chas_pwr_buffer_;
  data[0] = (uint8_t)(tmp_uint16 >> 8);
  data[1] = (uint8_t)(tmp_uint16);

  tmp_uint16 = (uint16_t)rfr_chas_pwr_limit_;
  data[2] = (uint8_t)(tmp_uint16 >> 8);
  data[3] = (uint8_t)(tmp_uint16);

  tmp_uint16 = (uint16_t)robot_hp_;
  data[4] = (uint8_t)(tmp_uint16 >> 8);
  data[5] = (uint8_t)(tmp_uint16);

  tmp_uint16 = (uint16_t)(req_pwr_ * 100.0f);
  data[6] = (uint8_t)(tmp_uint16 >> 8);
  data[7] = (uint8_t)(tmp_uint16);

  return true;
};
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace cap
}  // namespace hello_world