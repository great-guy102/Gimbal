/**
 * @file      : power_limiter.hpp
 * @brief     :
 *@author     : Yang Zhou (3200105353@zju.edu.cn)
 * @history   :
 *@date       : 2024-08-28
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 *  All Rights Reserved.
 *
 *******************************************************************************
 *| Version | Date | Author | Description |
 *******************************************************************************
 *******************************************************************************
 *@par last editor : Yang Zhou (3200105353@zju.edu.cn)
 *@par last edit time : 2024-08-28
 * @attention :
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_POWER_LIMITER_HPP_
#define HW_COMPONENTS_ALGORITHMS_POWER_LIMITER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "assert.hpp"
#include "basic_pid.hpp"
#include "controller_base.hpp"
#include "list.hpp"
#include "pid.hpp"
#include "stddef.h"

namespace hello_world
{
namespace power_limiter
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
enum MotorType : uint8_t {
  kMotor3508 = 0u,  ///< 3508 电机
  kMotor6020 = 1u,  ///< 6020 电机
  kMotorTypeNum     ///< 电机种类数量
};

enum PwrLimitCapMode : uint8_t {
  kPwrLimitSuperCapOff = 0u,     ///< 超级电容关闭，不使用超级电容
  kPwrLimitSuperCapNormal = 1u,  ///< 超级电容正常工作模式
  kPwrLimitSuperCapBoost = 2u,   ///< 超级电容快速放电模式
};
/* Exported types ------------------------------------------------------------*/

struct PwrLimitStaticParams {
  uint16_t z_ref_rfr = 20;            ///< 裁判系统功率控制下的缓冲能量收敛值
  uint16_t z_ref_cap = 20;            ///< 超级电容功率控制下的超电容量收敛值
  uint16_t z_danger = 5;              ///< 危险能量值，主要用于裁判系统功率控制
  float p_bias;                       ///< 底盘静息功率
  float pref_kd = 0;                  ///< 期望功率PID微分系数
  uint16_t pwr_permission_max = 800;  ///< 可用功率最大值，默认800w
};

struct MotorStaticParams {
  float R = 0;                  ///< 电机铜损
  float ka = 0;                 ///< 电机加速度系数
  float kv = 0;                 ///< 电机转速系数
  float km = 0;                 ///< 电机转矩系数
  MotorType type = kMotor3508;  ///< 电机类型
};

struct MotorRuntimeParams {
  float spd_measure_radps;  ///< 电机反馈实际转速 unit: rad/s
  float iq_measure;         ///< 电机反馈实际电流 unit: A
  float spd_ref_radps;      ///< 电机期望转速 unit: rad/s
  float iq_ref;             ///< 电机期望电流 unit: A
  MotorType type;           ///< 电机类型
};

struct PwrLimitRuntimeParams {
  bool is_referee_online;          ///< 裁判系统是否在线, 不在线则使用最保守的功率限制方案
  bool is_super_cap_online;        ///< 超级电容系统是否在线，不在线优先使用裁判系统下的功率限制
  PwrLimitCapMode super_cap_mode;  ///< 超级电容系统工作模式
  uint16_t p_rfr_max;              ///< 裁判系统给出的最大功率限制
  uint16_t z_rfr_measure;          ///< 裁判系统测量的缓冲区能量
  float p_rfr_measure;             ///< 裁判系统测量的底盘功率
  uint16_t p_dummy_max;            ///< 裁判系统给出的最大功率限制，TODO 未来可以配合超电更改
  uint16_t z_dummy_measure;        ///< 超电剩余容量，用于超电在线下的功率限制
  float p_cap_measure;             ///< 超级电容测量的底盘功率
};

struct PwrLimitCalcData {
  /* derived data and manual set references */
  float p_model;       ///< 功率限制模型建模功率
  float p_prediction;  ///< 功率限制预测功率
  float p_ref;         ///< 动态功率最大上限
  float k_limit;       ///< 速度削减系数
};

class PowerLimiter : public MemMgr
{
 public:
  typedef tools::list<MotorRuntimeParams> MotorRuntimeParamsList;
  typedef tools::list<MotorStaticParams> MotorStaticParamsList;

  PowerLimiter() = default;
  /**
   * @brief       构造函数，初始化功率限制静态参数
   * @param        &motor_static_list: 不同种类电机的静态参数，根据需要初始化，目前主要有3508和6020两种电机
   * @param        &pwr_limit_static_params: 功率限制静态参数
   * @retval       None
   * @arg         None
   * @note        None
   */
  PowerLimiter(const MotorStaticParamsList &motor_static_list, const PwrLimitStaticParams &pwr_limit_static_params);
  ~PowerLimiter()
  {
    if (p_ref_pid_ != nullptr) {
      delete p_ref_pid_;
      p_ref_pid_ = nullptr;
    }
    if (last_act_ref_spd_ != nullptr) {
      delete[] last_act_ref_spd_;
      last_act_ref_spd_ = nullptr;
    }
    if (spd_ref_radps_record_ != nullptr) {
      delete[] spd_ref_radps_record_;
      spd_ref_radps_record_ = nullptr;
    }
  };

  /**
   * @brief       初始化功率限制静态参数（可选调用）
   * @param        &motor_static_list: 不同种类电机的静态参数，根据需要初始化，目前主要有3508和6020两种电机
   * @param        &pwr_limit_static_params: 功率限制静态参数
   * @retval       None
   * @arg         None
   * @note        None
   */
  void Init(const MotorStaticParamsList &motor_static_list, const PwrLimitStaticParams &pwr_limit_static_params);

  /**
   * @brief       得到底盘轮数
   * @param        motor_par_list_: 电机运动参数列表
   * @retval       None
   * @arg         None
   * @note        None
   */
  size_t MotorNum(void) const { return motor_num_; }
  /**
   * @brief       获取功率限制计算出的相关数据--供调参等使用
   * @arg         None
   * @note        None
   */
  const PwrLimitCalcData &getData(void) { return pwr_limit_calc_data_; }
  /**
   * @brief        更改功率限制静态参数
   * @param        &pwr_limit_static_params: 修改后的静态参数，主要应用于以后的功率限制策略优化
   * @retval       None
   * @arg         None
   * @note        None
   */
  void setStaticParams(const PwrLimitStaticParams &pwr_limit_static_params);

  /**
   * @brief        更新功率限制运行参数
   * @param        *params: 功率限制运行时参数，主要包含裁判系统等信息
   * @param        motor_par_list: 各个电机的运行时参数
   * @retval       None
   * @arg         None
   * @note        None
   */
  void PwrLimitUpdateRuntimeParams(const PwrLimitRuntimeParams &params);

  /**
   * @brief         计算限制后各个电机的速度以控制底盘功率
   * @param        *limited_spd_ref_radps: 限制后的各轮参考速度指针，顺序与输入的电机运行参数顺序一致
   * @retval       None
   * @arg         None
   * @note        None
   */
  void PwrLimitCalcSpd(const MotorRuntimeParamsList &motor_run_par_list, float *limited_spd_ref_radps);

 private:
  typedef hello_world::pid::BasicPid PwrPid;
  /**
   * @brief       动态功率上限计算
   * @param        *pid: 动态功率计算PID实例指针
   * @param        mode: 超电供电模式
   * @param        p_max: 裁判系统功率限制
   * @param        z_ref: 能量收敛值
   * @param        z_measure: 能量测量值
   * @retval       None
   * @arg         None
   * @note        None
   */
  float PwrLimitCalcPref(PwrPid *pid, PwrLimitCapMode mode, float p_max, float z_ref, float z_measure);
  MotorRuntimeParamsList motor_run_par_list_ = MotorRuntimeParamsList();  ///< 电机运行时参数列表，储存各个电机的运行时参数
  MotorStaticParamsList motor_static_list_ = MotorStaticParamsList();     ///< 电机静态参数列表，储存每种电机的静态参数，暂时只有3508和6020两种
  PwrLimitCalcData pwr_limit_calc_data_;                                  ///< 功率限制计算数据
  PwrLimitStaticParams pwr_limit_static_params_;                          ///< 功率限制静态参数, 用于初始化功率限制算法,或者使用set方法进行修改,主要可能修改超电收敛值
  PwrLimitRuntimeParams pwr_limit_runtime_params_;                        ///< 记录来自于裁判系统或者超电的运行参数
  size_t motor_num_ = 0;                                                  ///< 电机数量
  PwrPid *p_ref_pid_ = nullptr;                                           ///< 动态功率PID
  MotorStaticParams motor_static_params_[kMotorTypeNum];                  ///< 固定两种电机类型，首位为3508，次位为6020，便于参数查询，此顺序与MotorType枚举相同，便于直接使用type查询
  float *last_act_ref_spd_ = nullptr;
  float *spd_ref_radps_record_ = nullptr;

};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace power_limiter
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_MULTI_NODES_PID_HPP_ */
