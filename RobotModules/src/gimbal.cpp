/**
 *******************************************************************************
 * @file      :gimbal.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "gimbal.hpp"
/* Private macro -------------------------------------------------------------*/
namespace robot {
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
hello_world::pid::MultiNodesPid::Datas pid_data; // TODO:PID调试数据
// 使用示例：pid_data = pid_ptr->getPidAt(0).datas();

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
#pragma region 数据更新

void Gimbal::update() {
  updateData();
  updatePwrState();
}

void Gimbal::updateData() {
  updateWorkTick();
  updateMotorData();
  updateImuData();
  updateIsPwrOn();
};

void Gimbal::updatePwrState() {
  // 无论任何状态，断电意味着要切到死亡状态
  if (!is_pwr_on_) {
    setPwrState(PwrState::kDead);
    return;
  }

  PwrState current_state = pwr_state_;
  PwrState next_state = current_state;
  if (current_state == PwrState::kDead) {
    // 死亡状态下，如果上电，则切到复活状态
    if (is_pwr_on_) {
      next_state = PwrState::kResurrection;
    }
  } else if (current_state == PwrState::kResurrection) {
    // 复活状态下，如果所有云台关节电机上电完毕，且云台板准备就绪，则切到工作状态
    // 都有云台板通讯告知
    // 1. 为什么要判断云台板准备就绪？
    // 云台板计算零飘完零飘后会告知底盘准备就绪，如果云台板还未准备好，云台关节就能够运动会导致云台板的姿态计算结果可能不准确
    // 2. 为什么要所有云台关节电机上电完毕，才认为云台模块准备好了？
    // 没啥原因，暂时先这么写着
    if (is_any_motor_pwron_) {
      next_state = PwrState::kWorking;
    }
  } else if (current_state == PwrState::kWorking) {
    // 工作状态下，保持当前状态
  } else {
    // 其他状态，认为是死亡状态
    next_state = PwrState::kDead;
  }
  setPwrState(next_state);
};

void Gimbal::updateMotorData() {
  Motor *motor_ptr = nullptr;
  JointIdx motor_idxs[2] = {kJointYaw, kJointPitch};

  bool is_any_motor_pwron = false;
  bool is_all_motor_pwron = true;

  for (size_t i = 0; i < 2; i++) {
    JointIdx motor_idx = motor_idxs[i];
    motor_ptr = motor_ptr_[motor_idx];
    HW_ASSERT(motor_ptr != nullptr, "pointer %d to motor %d is nullptr",
              motor_ptr, motor_idx);
    if (motor_ptr->isOffline()) {
      motor_ang_fdb_[motor_idx] = 0.0f;
      motor_spd_fdb_[motor_idx] = 0.0f;

      is_all_motor_pwron = false;
    } else {
      is_any_motor_pwron = true;

      motor_ang_fdb_[motor_idx] = motor_ptr->angle();
      // motor_spd_fdb_[joint_idx] = motor_ptr->spd();
      // 达妙速度反馈噪声大，使用td滤波计算速度
      td_motor_spd_ptr_[motor_idx]->calc(&motor_ang_fdb_[motor_idx],
                                         &motor_spd_fdb_[motor_idx]);
    }
  }

  is_any_motor_pwron_ = is_any_motor_pwron;
  is_all_motor_pwron_ = is_all_motor_pwron;
};

void Gimbal::updateIsPwrOn() {
  is_pwr_on_ = is_any_motor_pwron_ || is_rfr_pwr_on_;
};

void Gimbal::updateImuData() {
  HW_ASSERT(imu_ptr_ != nullptr, "pointer %d to imu %d is nullptr", imu_ptr_);
  // IMU是右手系，但pitch轴直觉上应该得是左手系，即低头角度为负，抬头角度为正，故在此处加负号
  imu_ang_fdb_[kJointYaw] = imu_ptr_->yaw();
  imu_ang_fdb_[kJointPitch] = (-1.0f) * imu_ptr_->pitch();

  imu_spd_fdb_[kJointYaw] = imu_ptr_->gyro_yaw();
  imu_spd_fdb_[kJointPitch] = (-1.0f) * imu_ptr_->gyro_pitch();
};

#pragma endregion

#pragma region 任务执行

void Gimbal::runOnDead() { resetDataOnDead(); };

void Gimbal::runOnResurrection() { resetDataOnResurrection(); };

void Gimbal::runOnWorking() {
  calcCtrlAngBased();
  adjustJointFdb();
  adjustLastJointAngRef();
  calcJointAngRef();
  calcJointTorRef();
};

void Gimbal::runAlways() { setCommData(pwr_state_ == PwrState::kWorking); }

void Gimbal::standby() {
  calcCtrlAngBased();
  adjustJointFdb();
  adjustLastJointAngRef();
  calcJointAngRef();

  setCommData(false);
};

void Gimbal::calcCtrlAngBased() {
  last_ctrl_ang_based_[kJointYaw] = ctrl_ang_based_[kJointYaw];
  last_ctrl_ang_based_[kJointPitch] = ctrl_ang_based_[kJointPitch];

  if (working_mode_ == WorkingMode::Normal) {
    ctrl_ang_based_[kJointYaw] = CtrlAngBased::Imu;
    ctrl_ang_based_[kJointPitch] = CtrlAngBased::Motor;
  } else if (working_mode_ == WorkingMode::PidTest) {
    ctrl_ang_based_[kJointYaw] = CtrlAngBased::Imu;
    ctrl_ang_based_[kJointPitch] = CtrlAngBased::Motor;
  } else {
    ctrl_ang_based_[kJointYaw] = CtrlAngBased::Imu;
    ctrl_ang_based_[kJointPitch] = CtrlAngBased::Motor;
  }
};

void Gimbal::adjustJointFdb() {
  JointIdx joint_idxs[kJointNum] = {kJointYaw, kJointPitch};
  for (size_t i = 0; i < kJointNum; i++) {
    JointIdx joint_idx = joint_idxs[i];
    if (ctrl_ang_based_[joint_idx] == CtrlAngBased::Motor) {
      joint_ang_fdb_[joint_idx] = motor_ang_fdb_[joint_idx];
    } else {
      joint_ang_fdb_[joint_idx] = imu_ang_fdb_[joint_idx];
    }
    joint_spd_fdb_[joint_idx] = imu_spd_fdb_[joint_idx];
  }
}

void Gimbal::adjustLastJointAngRef() {
  // 判断工作模式是否发生变化
  JointIdx joint_idxs[2] = {kJointYaw, kJointPitch};
  for (size_t i = 0; i < 2; i++) {
    JointIdx joint_idx = joint_idxs[i];
    if (last_ctrl_ang_based_[joint_idx] == ctrl_ang_based_[joint_idx]) {
      last_joint_ang_ref_[joint_idx] = joint_ang_ref_[joint_idx];
      continue; // 同一模式下，沿用前一次的控制指令
    }

    if (ctrl_ang_based_[joint_idx] == CtrlAngBased::Motor) {
      last_joint_ang_ref_[joint_idx] =
          motor_ang_fdb_[joint_idx]; // 切换到电机控制时，以电机角度为准重置
    } else if (ctrl_ang_based_[joint_idx] == CtrlAngBased::Imu) {
      last_joint_ang_ref_[joint_idx] =
          imu_ang_fdb_[joint_idx]; // 切换到IMU控制时，以IMU角度为准重置
    } else {
      HW_ASSERT(false, "unknown ctrl_ang_based_[joint_idx] %d",
                ctrl_ang_based_[joint_idx]);
    }
  }
}

void Gimbal::calcJointAngRef() {
  // 如果控制模式是自动，且视觉模块没有离线、视觉模块检测到有效目标，且视觉反馈角度与当前角度相差不大
  GimbalCmd tmp_ang_ref = {0.0f}, raw_ang_ref = {0.0f};

  static bool first_sentry_flag = true;
  if (ctrl_mode_ == CtrlMode::kAuto && vis_data_.is_target_detected &&
      fabsf(joint_ang_fdb_[kJointYaw] - vis_data_.cmd.yaw) < 0.3927f &&
      fabsf(joint_ang_fdb_[kJointPitch] - vis_data_.cmd.pitch) < 0.30543f) {
    tmp_ang_ref = vis_data_.cmd;
  } else {
    float yaw_angle_delta = 0.0f;
    float pitch_angle_delta = 0.0f;
    static bool pid_mode_refreshed = true;
    const float delta_upper_limit = 0.99;
    const float delta_lower_limit = 0.50;

    switch (working_mode_) {
    case WorkingMode::Normal: {
      first_sentry_flag = true;

      float sensitivity_yaw =
          cfg_.sensitivity_yaw; // yaw角度灵敏度，单位 rad/ms
      float sensitivity_pitch =
          cfg_.sensitivity_pitch; // pitch角度灵敏度，单位 rad/ms
      if (rev_gimbal_flag_ && work_tick_ - last_rev_head_tick_ > 400) {
        yaw_angle_delta = PI;
        last_rev_head_tick_ = work_tick_;
      } else if (!rev_gimbal_flag_ && work_tick_ - last_rev_head_tick_ > 400) {
        yaw_angle_delta = norm_cmd_delta_.yaw * sensitivity_yaw;
      }
      pitch_angle_delta = norm_cmd_delta_.pitch * sensitivity_pitch;
      tmp_ang_ref.yaw = last_joint_ang_ref_[kJointYaw] + yaw_angle_delta;
      tmp_ang_ref.pitch = last_joint_ang_ref_[kJointPitch] + pitch_angle_delta;
    } break;

    case WorkingMode::Sentry: {
      const float theta_pitch_upper_limit = 0.10f;
      const float theta_pitch_lower_limit = -0.07f;
      const float theta_pitch_frequency =
          8.0f / 1000.0f; // 哨兵模式pitch正弦轨迹检索频率
      const float theta_pitch_peak =
          (theta_pitch_upper_limit - theta_pitch_lower_limit) / 2.0f;
      const float theta_pitch_mid =
          (theta_pitch_upper_limit + theta_pitch_lower_limit) / 2.0f;

      static float work_tick_sentry_initial = 0.0f;
      static float theta_initial = 0.0f;
      static float phase_initial = 0.0f;
      if (first_sentry_flag) {
        first_sentry_flag = false;
        work_tick_sentry_initial = work_tick_;
        theta_initial = hello_world::Bound(joint_ang_fdb_[kJointPitch],
                                           theta_pitch_lower_limit,
                                           theta_pitch_upper_limit);
        phase_initial =
            asinf((theta_initial - theta_pitch_mid) / theta_pitch_peak);
      }
      // θ(t) = A * sin(w(t-t0) + φ) + θ(0)
      // dθ(t) = A*w * cos(w(t-t0) + φ)
      // pitch_angle_delta =
      //     theta_pitch_peak * theta_pitch_frequency *
      //     arm_cos_f32(theta_pitch_frequency *
      //                     (work_tick_ - work_tick_sentry_initial) +
      //                 phase_initial);
      raw_ang_ref.pitch =
          theta_pitch_mid +
          theta_pitch_peak *
              arm_sin_f32(theta_pitch_frequency *
                              (work_tick_ - work_tick_sentry_initial) +
                          phase_initial);
      tmp_ang_ref.pitch = raw_ang_ref.pitch;
      // TODO:对于pitch关节大幅度阶跃，考虑限制变化率，抑制震荡
      // ramp_joint_v_ptr_[kJointPitch]->calc(&(raw_ang_ref.pitch),
      // &(tmp_ang_ref.pitch));

      yaw_angle_delta = 0.5f * cfg_.sensitivity_yaw;
      tmp_ang_ref.yaw = last_joint_ang_ref_[kJointYaw] + yaw_angle_delta;
    } break;
    case WorkingMode::PidTest: {
      first_sentry_flag = true;

      const float angle_yaw_step_delta = hello_world::Deg2Rad(10);
      const float angle_pitch_step_delta = hello_world::Deg2Rad(5);
      if (fabs(norm_cmd_delta_.yaw) >= delta_upper_limit &&
          pid_mode_refreshed) {
        pid_mode_refreshed = false;
        if (norm_cmd_delta_.yaw >= delta_upper_limit) {
          yaw_angle_delta = angle_yaw_step_delta;
        } else if (norm_cmd_delta_.yaw <= -delta_upper_limit) {
          yaw_angle_delta = (-1.0) * angle_yaw_step_delta;
        }
      } else if (fabs(norm_cmd_delta_.pitch) >= delta_upper_limit &&
                 pid_mode_refreshed) {
        pid_mode_refreshed = false;
        if (norm_cmd_delta_.pitch >= delta_upper_limit) {
          pitch_angle_delta = angle_pitch_step_delta;
        } else if (norm_cmd_delta_.pitch <= -delta_upper_limit) {
          pitch_angle_delta = (-1.0) * angle_pitch_step_delta;
        }
      } else if ((fabs(norm_cmd_delta_.yaw) <= delta_lower_limit) &&
                 (fabs(norm_cmd_delta_.pitch) <= delta_lower_limit) &&
                 !pid_mode_refreshed) {
        pid_mode_refreshed = true;
      }
      tmp_ang_ref.yaw = last_joint_ang_ref_[kJointYaw] + yaw_angle_delta;
      tmp_ang_ref.pitch = last_joint_ang_ref_[kJointPitch] + pitch_angle_delta;
    } break;

    default:
      HW_ASSERT(false, "Unknown WorkingMode %d", working_mode_);
      break;
    }
  }

  tmp_ang_ref.yaw = hello_world::AngleNormRad(tmp_ang_ref.yaw);
  tmp_ang_ref.pitch = hello_world::AngleNormRad(tmp_ang_ref.pitch);

  // Pitch Axis Limits
  if (ctrl_ang_based_[kJointPitch] == CtrlAngBased::Motor) {
    tmp_ang_ref.pitch = hello_world::Bound(
        tmp_ang_ref.pitch, cfg_.min_pitch_ang, cfg_.max_pitch_ang);
  } else if (ctrl_ang_based_[kJointPitch] == CtrlAngBased::Imu) {
    float motor_imu_delta =
        imu_ang_fdb_[kJointPitch] - motor_ang_fdb_[kJointPitch];
    tmp_ang_ref.pitch = hello_world::Bound(
        tmp_ang_ref.pitch, cfg_.min_pitch_ang + motor_imu_delta,
        cfg_.max_pitch_ang + motor_imu_delta);
  } else {
    HW_ASSERT(false, "unknown ctrl_ang_based_[kJointPitch] %d",
              ctrl_ang_based_[kJointPitch]);
  }
  // Update Joint Angle References
  joint_ang_ref_[kJointYaw] = tmp_ang_ref.yaw;
  joint_ang_ref_[kJointPitch] = tmp_ang_ref.pitch;
}

void Gimbal::calcJointTorRef() {
  JointIdx joint_idxs[kJointNum] = {kJointPitch, kJointYaw};
  for (uint8_t i = 0; i < kJointNum; i++) {
    JointIdx joint_idx = joint_idxs[i];
    Pid *pid_ptr = pid_ptr_[joint_idx];
    float ref[2] = {joint_ang_ref_[joint_idx], 0.0f};
    float fdb[2] = {joint_ang_fdb_[joint_idx], joint_spd_fdb_[joint_idx]};
    float forward_toq[2] = {0.0f, 0.0f};

    // pitch轴重力前馈
    if (joint_idx == kJointPitch) {
      float pitch_tor_k = 3.0f; //  (arccos(1/5) / max_pitch_ang)（单位：rad）
      forward_toq[0] =
          0.75f * arm_cos_f32(pitch_tor_k * joint_ang_ref_[joint_idx]);
    }

    // yaw轴阻力前馈
    if (joint_idx == kJointYaw) {
      if ((joint_ang_ref_[joint_idx] - joint_ang_fdb_[joint_idx]) > 0.01f) {
        forward_toq[1] = 0.3f;
      } else if ((joint_ang_ref_[joint_idx] - joint_ang_fdb_[joint_idx]) <
                 -0.01f) {
        forward_toq[1] = -0.2f;
      } else {
        forward_toq[1] = 0.0f;
      }
    }

    HW_ASSERT(pid_ptr != nullptr, "pointer to PID %d is nullptr", joint_idx);
    pid_ptr->calc(ref, fdb, forward_toq, &joint_tor_ref_[joint_idx]);
  }
}

#pragma endregion

#pragma region 数据重置

/**
 * @brief 重置除指针之外的所有数据，使状态机回到初始状态
 */
void Gimbal::reset() {
  pwr_state_ = PwrState::kDead; ///< 工作状态

  // 在 runOnWorking 函数中更新的数据
  ctrl_mode_ = CtrlMode::kManual;      ///< 控制模式
  working_mode_ = WorkingMode::Normal; ///< 工作模式

  memset(ctrl_ang_based_, (int)CtrlAngBased::Imu, sizeof(ctrl_ang_based_));

  memset(joint_ang_ref_, 0, sizeof(joint_ang_ref_)); ///< 控制指令，基于关节空间
  memset(joint_ang_fdb_, 0, sizeof(joint_ang_fdb_)); ///< 云台关节角度反馈值
  memset(joint_spd_fdb_, 0, sizeof(joint_spd_fdb_)); ///< 云台关节速度反馈值
  memset(last_joint_ang_ref_, 0,
         sizeof(last_joint_ang_ref_)); ///< 上一次控制指令，基于关节空间
  memset(joint_tor_ref_, 0, sizeof(joint_tor_ref_)); ///< 控制指令，基于关节力矩

  // 从电机中拿的数据
  is_any_motor_pwron_ = false; ///< 任意电机是否处于就绪状态
  is_all_motor_pwron_ = false; ///< 所有电机是否都处于就绪状态

  memset(motor_ang_fdb_, 0,
         sizeof(motor_ang_fdb_)); ///< 云台关节角度反馈值【电机】
  memset(motor_spd_fdb_, 0,
         sizeof(motor_spd_fdb_)); ///< 云台关节速度反馈值【电机】

  // 从IMU中拿的数据
  memset(imu_ang_fdb_, 0, sizeof(imu_ang_fdb_)); ///< 云台关节角度反馈值【IMU】
  memset(imu_spd_fdb_, 0, sizeof(imu_spd_fdb_)); ///< 云台关节速度反馈值【IMU】

  resetPids();
};

void Gimbal::resetDataOnDead() {
  // 在 runOnWorking 函数中更新的数据
  ctrl_mode_ = CtrlMode::kManual;      ///< 控制模式
  working_mode_ = WorkingMode::Normal; ///< 工作模式

  memset(ctrl_ang_based_, (int)CtrlAngBased::Imu, sizeof(ctrl_ang_based_));

  memset(joint_ang_ref_, 0, sizeof(joint_ang_ref_)); ///< 控制指令，基于关节空间
  memset(joint_ang_fdb_, 0, sizeof(joint_ang_fdb_)); ///< 云台关节角度反馈值
  memset(joint_spd_fdb_, 0, sizeof(joint_spd_fdb_)); ///< 云台关节速度反馈值
  memset(last_joint_ang_ref_, 0,
         sizeof(last_joint_ang_ref_)); ///< 上一次控制指令，基于关节空间
  memset(joint_tor_ref_, 0, sizeof(joint_tor_ref_)); ///< 控制指令，基于关节力矩

  resetPids();
}

void Gimbal::resetDataOnResurrection() {
  // 在 runOnWorking 函数中更新的数据
  ctrl_mode_ = CtrlMode::kManual;      ///< 控制模式
  working_mode_ = WorkingMode::Normal; ///< 工作模式

  memset(ctrl_ang_based_, (int)CtrlAngBased::Imu, sizeof(ctrl_ang_based_));

  memset(joint_ang_ref_, 0, sizeof(joint_ang_ref_)); ///< 控制指令，基于关节空间
  memset(joint_ang_fdb_, 0, sizeof(joint_ang_fdb_)); ///< 云台关节角度反馈值
  memset(joint_spd_fdb_, 0, sizeof(joint_spd_fdb_)); ///< 云台关节速度反馈值
  memset(last_joint_ang_ref_, 0,
         sizeof(last_joint_ang_ref_)); ///< 上一次控制指令，基于关节空间
  memset(joint_tor_ref_, 0, sizeof(joint_tor_ref_)); ///< 控制指令，基于关节力矩

  resetPids();
}

void Gimbal::resetPids() {
  for (size_t i = 0; i < kJointNum; i++) {
    HW_ASSERT(pid_ptr_[i] != nullptr, "pointer to PID %d is nullptr", i);
    pid_ptr_[i]->reset();
  }
};
#pragma endregion

#pragma region 通信数据设置

void Gimbal::setCommDataMotors(bool working_flag) {
  // 机器人工作时
  // 电机根据期望电流输入发送数据
  JointIdx joint_idxs[2] = {kJointYaw, kJointPitch};
  for (size_t i = 0; i < 2; i++) {
    JointIdx joint_idx = joint_idxs[i];
    HW_ASSERT(motor_ptr_[joint_idx] != nullptr,
              "pointer to motor %d is nullptr", joint_idx);
    if (working_flag && (!motor_ptr_[joint_idx]->isOffline())) {
      motor_ptr_[joint_idx]->setInput(joint_tor_ref_[joint_idx]);
      // if (joint_idx == kJointYaw) // TODO调试
      // {
      //   // motor_ptr_[joint_idx]->setInput(joint_tor_ref_[joint_idx]);
      //   motor_ptr_[joint_idx]->setInput(0);
      // } else {
      //   // motor_ptr_[joint_idx]->setInput(joint_tor_ref_[joint_idx]);
      //   motor_ptr_[joint_idx]->setInput(0);
      // }
    } else {
      pid_ptr_[joint_idx]->reset();
      motor_ptr_[joint_idx]->setInput(0);
    }
  }
};
#pragma endregion

#pragma region 注册函数

void Gimbal::registerMotor(Motor *ptr, JointIdx idx) {
  HW_ASSERT(ptr != nullptr, "pointer to motor %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kJointNum, "index of motor out of range", idx);
  motor_ptr_[idx] = ptr;
};
void Gimbal::registerPid(Pid *ptr, JointIdx idx) {
  HW_ASSERT(ptr != nullptr, "pointer to PID %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kJointNum, "index of PID out of range", idx);
  pid_ptr_[idx] = ptr;
};
void Gimbal::registerImu(Imu *ptr) {
  HW_ASSERT(ptr != nullptr, "pointer to imu is nullptr", ptr);
  imu_ptr_ = ptr;
}

void Gimbal::registerTd(Td *ptr, size_t idx) {
  HW_ASSERT(ptr != nullptr, "pointer to Td is nullptr", ptr);
  HW_ASSERT(idx < kJointNum, "index of Td out of range", idx);
  td_motor_spd_ptr_[idx] = ptr;
}

void Gimbal::registerRamp(Ramp *ptr, size_t idx) {
  HW_ASSERT(ptr != nullptr, "pointer to Ramp is nullptr", ptr);
  HW_ASSERT(idx < kJointNum, "index of Ramp out of range", idx);
  ramp_joint_v_ptr_[idx] = ptr;
}

#pragma endregion

#pragma region 其他工具函数

#pragma endregion
/* Private function definitions ----------------------------------------------*/

} // namespace robot