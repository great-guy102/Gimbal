/**
 *******************************************************************************
 * @file      :ins_fric.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.1.0      2025-1-15       Wpy             Not yet
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team,Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/

#include "ins_fric.hpp"
#include "ins_motor.hpp"
#include "ins_pid.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
// TODO: 修改配置参数
hw_module::Fric::Config kFricConfig = {
    .default_spd_ref = 650.0f, // 摩擦轮期望速度预设值, >0, 无默认值, rad/s
    .default_spd_ref_backward =
        -100.0f, // 摩擦轮反转目标速度, <0, 默认值 -100 rad/s,
                 // 反转模式是为了将卡在摩擦轮中间的弹丸回退出来，转速不易过快
    .stuck_curr_thre = 14.0f, // 用于判断摩擦轮堵转的电流阈值, >0, 默认值 14 A
    .spd_delta_thre =
        10.0f, // 用于判断摩擦轮速度保持恒定的阈值, >0, 默认值 10 rad/s
    .spd_err_thre =
        5.0f, // 用于判断摩擦轮速度跟上期望转速的阈值, >0, 默认值 5 rad/s
    .spd_stop_thre = 30.0f, // 摩擦轮Stop模式，转速小于该阈值后，停止控制电机,
                            // >0, 默认值 100 rad/s
    /* 优化项，建议开启 */
    .opt_spd_same_pid_enabled =
        false, // 是否使用双摩擦轮同速PID(期望为0，反馈输入为两轮差速，输出分别正负作用到两个电机上),开启时需要注册对应的PID,否则会进入断言错误
    .opt_blt_spd_cl =
        {
            .is_enabled = true, // TODO：是否开启弹速闭环，
            .min_reasonable_blt_spd =
                15.0f, // 最小合理弹丸速度, >0, 无默认值, m/s,
                       // 小于该值认为裁判系统反馈数据错误
            .max_reasonable_blt_spd =
                30.0f, // 最大合理弹丸速度, >0, 无默认值, m/s,
                       // 大于该值认为裁判系统反馈数据错误
            .min_target_blt_spd =
                21.8f, // 弹丸速度期望值区间下限, >0, 无默认值, m/s
            .max_target_blt_spd =
                22.3f,            // 弹丸速度期望值区间上限, >0, 无默认值, m/s
            .spd_gradient = 3.0f, // 摩擦轮转速调整梯度, >0, 默认值 5 rad/s
        },                        // 弹速闭环优化
};

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
hw_module::Fric unique_fric = hw_module::Fric(kFricConfig);
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
hw_module::Fric *GetFric() {
  static bool is_fric_created = false;
  if (!is_fric_created) {
    unique_fric.registerPid(
        GetPidMotorFricLeft(),
        hw_module::Fric::PidIdx::kFirst); // 注册MultiNodesPID指针
    unique_fric.registerPid(
        GetPidMotorFricRight(),
        hw_module::Fric::PidIdx::kSecond); // 注册MultiNodesPID指针
    unique_fric.registerMotor(
        GetMotorFricLeft(),
        hw_module::Fric::MotorIdx::kFirst); // 注册电机指针
    unique_fric.registerMotor(
        GetMotorFricRight(),
        hw_module::Fric::MotorIdx::kSecond); // 注册电机指针
    is_fric_created = true;
  }
  return &unique_fric;
};
