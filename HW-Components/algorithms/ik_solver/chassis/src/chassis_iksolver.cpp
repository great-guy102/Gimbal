/**
 * @file      chassis_iksolver.cpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2023-12-13
 * @brief     基于约束的轮式移动机器人运动学逆求解器
 * @details   标准轮是各类车轮(转向轮、swedish轮等)的基础，且可以理想化为一个圆形。通过该圆形施加滚动约束和无侧滑约束，可以求解出满足底盘坐标系下速度指令的各个车轮的转速。
 * @par last edit time  2024-02-24
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @version   2.1.1
 *
 * @copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All Rights Reserved.
 *
 * @attention 只考虑底盘轮组与单一平面接触的情况
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 2.0.0 | 2024-01-24 | ZhouShichan | 1. 首次完成 |
 * | 2.1.0 | 2024-02-11 | ZhouShichan | 对求解方式进行了优化，改变了容器实现方式，更改部分接口函数 |
 * | 2.1.1 | 2024-02-13 | ZhouShichan | 修复固定标准轮、Swedish 轮求解异常；修复 Swedish 轮 gamma 值初始化异常； |
 * | 2.1.2 | 2024-02-24 | ZhouShichan | ChassisIKSolver 的成员函数 getThetaVelRefAll getIsNoSideSlipAll 获取数据异常 |
 *
 * @par v2.1.1
 * 1. 修复固定标准轮、Swedish 轮求解异常
 * 2. 修复 Swedish 轮 gamma 值初始化异常
 *
 * @par v2.1.0
 * 1. 各种轮子的求解方式相互独立，对转向标准轮和球轮的求解添加了转角优化
 * 2. 链表容器更改为 std::list ，在 Ozone 中不再支持直接查看内部数据，需要通过额外的全局变量查看
 * 3. ChassisIKSolver 添加底盘旋转中心的设置函数，求解函数不再支持通过指针返回逆解结果，需要通过其他函数单独获取
 *
 * @par 相关链接
 * [内部飞书](https://g6ursaxeei.feishu.cn/wiki/wikcnob2XRghAIPINsoeG6AYy5f)
 * [Github Wiki](https://zju-helloworld.github.io/Wiki/%E7%BB%84%E4%BB%B6%E8%AF%B4%E6%98%8E/%E6%9C%BA%E5%99%A8%E4%BA%BA%E9%80%9A%E7%94%A8%E7%BB%84%E4%BB%B6/%E7%AE%97%E6%B3%95/%E8%BD%AE%E5%BC%8F%E6%9C%BA%E5%99%A8%E4%BA%BA%E5%BA%95%E7%9B%98%E7%BA%A6%E6%9D%9F%E6%B1%82%E8%A7%A3/) 推荐
 */
/* Includes ------------------------------------------------------------------*/

#include "chassis_iksolver.hpp"

#include "base.hpp"
#include "system.hpp"

namespace hello_world
{
namespace chassis_ik_solver
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
HW_OPTIMIZE_O2_START

ChassisIkSolver& ChassisIkSolver::operator=(const ChassisIkSolver& other)
{
  if (this == &other) {
    return *this;
  }

  control_center_ = other.control_center_;
  vel_r_ = other.vel_r_;
  wheel_list_ = other.wheel_list_;

  return *this;
}

ChassisIkSolver::ChassisIkSolver(ChassisIkSolver&& other)
{
  control_center_ = std::move(other.control_center_);
  vel_r_ = std::move(other.vel_r_);
  wheel_list_ = std::move(other.wheel_list_);
}

ChassisIkSolver& ChassisIkSolver::operator=(ChassisIkSolver&& other)
{
  if (this == &other) {
    return *this;
  }

  control_center_ = std::move(other.control_center_);
  vel_r_ = std::move(other.vel_r_);
  wheel_list_ = std::move(other.wheel_list_);

  return *this;
}

IkSolveStatus ChassisIkSolver::solve(
    const MoveVec& v, float* theta_vel_fdbs_ptr)
{
  IkSolveStatus status = kIkSolveStatusOk;

  size_t idx = 0;

  vel_r_ = v;

  for (auto& wheel : wheel_list_) {
    IkSolveRes res;

    float* theta_vel_fdb_ptr = theta_vel_fdbs_ptr == nullptr
                                   ? nullptr
                                   : theta_vel_fdbs_ptr + idx;

    IkSolveStatus wheel_status = wheel->ikSolve(v, &res, theta_vel_fdb_ptr);

    SetBits(status, wheel_status);

    idx++;
  }
  return status;
}

IkSolveStatus ChassisIkSolver::solve(
    const MoveVec& v, float theta_i2r, float* theta_vel_fdbs_ptr)
{
  IkSolveStatus status = kIkSolveStatusOk;

  MoveVec v_r;
  v.rotate(theta_i2r, &v_r);

  size_t idx = 0;

  vel_r_ = v_r;

  for (auto& wheel : wheel_list_) {
    IkSolveRes res;

    float* theta_vel_fdb_ptr = theta_vel_fdbs_ptr == nullptr
                                   ? nullptr
                                   : theta_vel_fdbs_ptr + idx;

    IkSolveStatus wheel_status = wheel->ikSolve(v_r, &res, theta_vel_fdb_ptr);

    SetBits(status, wheel_status);

    idx++;
  }
  return status;
}

bool ChassisIkSolver::append(WheelType wheel_type, const WheelParams& params)
{
  Wheel* wheel_ptr = CreateWheel(wheel_type, params);
  if (wheel_ptr == nullptr) {
    return false;
  } else {
    wheel_ptr->setCenterPos(&control_center_);
    wheel_list_.push_back(wheel_ptr);
    return true;
  }
}

void ChassisIkSolver::erase_tail(void)
{
  wheel_list_.pop_back();
}

void ChassisIkSolver::clear(void)
{
  wheel_list_.clear();
}

const Wheel* ChassisIkSolver::getWheel(size_t idx) const
{
  if (idx >= size()) {
    return nullptr;
  }

  auto iter = wheel_list_.begin();
  for (size_t i = 0; i < idx; i++) {
    ++iter;
  }
  return *iter;
}

Wheel* ChassisIkSolver::getWheel(size_t idx)
{
  if (idx >= size()) {
    return nullptr;
  }

  auto iter = wheel_list_.begin();
  for (size_t i = 0; i < idx; i++) {
    ++iter;
  }
  return *iter;
}

void ChassisIkSolver::setCenterPos(float x, float y)
{
  control_center_.x() = x;
  control_center_.y() = y;
  for (auto& wheel : wheel_list_) {
    wheel->setCenterPos(&control_center_);
  }
}

const IkSolveRes& ChassisIkSolver::getIkSolveRes(size_t idx) const
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(
      size() > 0,
      "ChassisIkSolver must has a wheel when use this function, but %d now",
      size());
#pragma endregion
  const Wheel* wheel_ptr = getWheel(idx);
  return wheel_ptr->getIkSolveRes();
}

bool ChassisIkSolver::getIsNoSideSlip(size_t idx) const
{
  return getIkSolveRes(idx).is_no_side_slip;
}

float ChassisIkSolver::getRotSpd(size_t idx) const
{
  return getIkSolveRes(idx).rot_spt;
}

float ChassisIkSolver::getThetaVelRef(size_t idx) const
{
  return getIkSolveRes(idx).theta_vel_ref;
}

IkSolveStatus ChassisIkSolver::getIkSolveResAll(
    IkSolveRes* iksolve_ress_ptr) const
{
  if (iksolve_ress_ptr == nullptr) {
    return kIkSolveStatusFailReturn;
  }
  for (auto& i : wheel_list_) {
    *iksolve_ress_ptr = i->getIkSolveRes();
    iksolve_ress_ptr++;
  }
  return kIkSolveStatusOk;
}

IkSolveStatus ChassisIkSolver::getRotSpdAll(float* rot_spds_ptr) const
{
  if (rot_spds_ptr == nullptr) {
    return kIkSolveStatusFailReturn;
  }
  for (auto& i : wheel_list_) {
    *rot_spds_ptr = i->getRotSpd();
    rot_spds_ptr++;
  }
  return kIkSolveStatusOk;
}

IkSolveStatus ChassisIkSolver::getThetaVelRefAll(
    float* theta_vel_refs_ptr) const
{
  if (theta_vel_refs_ptr == nullptr) {
    return kIkSolveStatusFailReturn;
  }
  for (auto& i : wheel_list_) {
    *theta_vel_refs_ptr = i->getThetaVelRef();
    theta_vel_refs_ptr++;
  }
  return kIkSolveStatusOk;
}

IkSolveStatus ChassisIkSolver::getIsNoSideSlipAll(
    bool* is_no_side_slips_ptr) const
{
  if (is_no_side_slips_ptr == nullptr) {
    return kIkSolveStatusFailReturn;
  }

  for (auto& i : wheel_list_) {
    *is_no_side_slips_ptr = i->getIsNoSideSlip();
    is_no_side_slips_ptr++;
  }
  return kIkSolveStatusOk;
}

/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace chassis_ik_solver
}  // namespace hello_world
