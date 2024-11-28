/**
 * @file      multi_nodes_pid.cpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-28
 * @brief
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 *
 * @copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All Rights Reserved.
 *
 * @attention
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2024-MM-DD | ZhouShichan | description |
 */
/* Includes ------------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
#include "multi_nodes_pid.hpp"

#include "assert.hpp"
#include "base.hpp"
namespace hello_world
{
namespace pid
{
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
HW_OPTIMIZE_O2_START

MultiNodesPid::MultiNodesPid(Type type, OutLimit out_limit)
{
  type_ = type;
  out_limit_ = out_limit;
  setControllerDims(0);
}
MultiNodesPid::MultiNodesPid(Type type, OutLimit out_limit, size_t size)
{
  type_ = type;
  out_limit_ = out_limit;
  pids_.resize(size);
  setControllerDims(size);
}
MultiNodesPid::MultiNodesPid(Type type, OutLimit out_limit, const PidList& pids)
{
  type_ = type;
  out_limit_ = out_limit;
  size_t size = 0;
  for (auto iter = pids.begin(); iter != pids.end(); ++iter, size++) {
    pids_.push_back(Pid(*iter));
  }
  setControllerDims(size);
}
MultiNodesPid::MultiNodesPid(Type type, OutLimit out_limit,
                             const ParamsList& params_list)
{
  type_ = type;
  out_limit_ = out_limit;
  size_t size = 0;
  for (auto iter = params_list.begin(); iter != params_list.end();
       ++iter, size++) {
    pids_.push_back(Pid(*iter));
  }
  setControllerDims(size);
}

MultiNodesPid& MultiNodesPid::operator=(const MultiNodesPid& other)
{
  if (this == &other) {
    return *this;
  }

  Controller::operator=(other);
  type_ = other.type_;
  out_limit_ = other.out_limit_;
  pids_ = other.pids_;
  calc_state_ = other.calc_state_;
  out_ = other.out_;
  return *this;
}

MultiNodesPid::MultiNodesPid(MultiNodesPid&& other)
    : Controller(std::move(other))
{
  type_ = other.type_;
  out_limit_ = other.out_limit_;
  pids_ = std::move(other.pids_);
  ref_dim_ = other.ref_dim_;
  fdb_dim_ = other.fdb_dim_;
  ffd_dim_ = other.ffd_dim_;
  out_dim_ = other.out_dim_;
  calc_state_ = other.calc_state_;
  out_ = other.out_;
}

MultiNodesPid& MultiNodesPid::operator=(MultiNodesPid&& other)
{
  if (this == &other) {
    return *this;
  }

  Controller::operator=(std::move(other));
  type_ = other.type_;
  out_limit_ = other.out_limit_;
  pids_ = std::move(other.pids_);
  calc_state_ = other.calc_state_;
  out_ = other.out_;
  return *this;
}

void MultiNodesPid::push_back(const Pid& pid)
{
  pids_.push_back(Pid(pid));
  setControllerDims(pids_.size());
}

void MultiNodesPid::push_back(const Params& params)
{
  pids_.push_back(Pid(params));
  setControllerDims(pids_.size());
}

void MultiNodesPid::remove(size_t idx)
{
  HW_ASSERT(idx < pids_.size(), "Illegal index %d", idx);
  auto iter = pids_.begin();
  for (size_t i = 0; i < idx; i++) {
    ++iter;
    if (iter == pids_.end()) {
      return;
    }
  }
  pids_.erase(iter);
  setControllerDims(pids_.size());
}

MultiNodesPid::Pid& MultiNodesPid::getPidAt(size_t idx)
{
  HW_ASSERT(idx < pids_.size(), "Illegal index %d", idx);
  auto iter = pids_.begin();
  for (size_t i = 0; i < idx; i++) {
    ++iter;
    if (iter == pids_.end()) {
      return *iter;
    }
  }
  return *iter;
}

const MultiNodesPid::Pid& MultiNodesPid::getPidAt(size_t idx) const
{
  HW_ASSERT(idx < pids_.size(), "Illegal index %d", idx);
  auto iter = pids_.begin();
  for (size_t i = 0; i < idx; i++) {
    ++iter;
    if (iter == pids_.end()) {
      return *iter;
    }
  }
  return *iter;
}

MultiNodesPid::State MultiNodesPid::reset(void)
{
  for (auto iter = pids_.begin(); iter != pids_.end(); ++iter) {
    iter->reset();
  }
  calc_state_ = kPidCalcStateNone;
  return kControllerStateOk;
}
MultiNodesPid::State MultiNodesPid::calc(
    const float ref_arr[], const float fdb_arr[],
    const float ffd_arr[], float out_arr[])
{
  if (type_ == Type::kCascade) {
    return calcCascade(ref_arr, fdb_arr, ffd_arr, out_arr);
  } else if (type_ == Type::kParallel) {
    return calcParallel(ref_arr, fdb_arr, ffd_arr, out_arr);
  } else {
    HW_ASSERT(0, "Illegal multi nodes pid type %d", type_);
  }
  return kControllerStateError;
}
MultiNodesPid::State MultiNodesPid::calcCascade(
    const float ref_arr[], const float fdb_arr[],
    const float ffd_arr[], float out_arr[])
{
  HW_ASSERT(type_ == Type::kCascade,
            "Illegal multi nodes pid type %d", type_);
  calc_state_ = kPidCalcStateNone;

  if (fdb_arr == nullptr) {
    out_ = 0;
    SetBits(kPidCalcStateLackFdb, calc_state_);
    return kControllerStateError;
  }

  float ref = 0;
  if (ref_arr != nullptr) {
    ref = ref_arr[0];
  } else {
    SetBits(kPidCalcStateLackRef, calc_state_);
  }

  float out = 0;
  float zero_ffd = 0;
  const float* zero_ffd_ptr = &zero_ffd;

  size_t i = 0;
  for (auto& pid : pids_) {
    // 由于 pid 会记录 前馈值 传入 nullptr ，在此传入一个指向 0 的值
    pid.calc(&ref, &fdb_arr[i], zero_ffd_ptr, &out);
    ref = out;
    i++;
  }

  if (ffd_arr == nullptr) {
    out_ = out_limit_.calc(out);
  } else {
    out_ = out_limit_.calc(out + ffd_arr[0]);
  }

  if (out_arr != nullptr) {
    out_arr[0] = out_;
  } else {
    SetBits(kPidCalcStateFailedOut, calc_state_);
  }

  SetBits(kPidCalcStateOk, calc_state_);
  return calc_state_ == kPidCalcStateOk
             ? kControllerStateOk
             : kControllerStateError;
}
MultiNodesPid::State MultiNodesPid::calcParallel(
    const float ref_arr[], const float fdb_arr[],
    const float ffd_arr[], float out_arr[])
{
  HW_ASSERT(type_ == Type::kParallel,
            "Illegal multi nodes pid type %d", type_);

  calc_state_ = CalcState::kPidCalcStateNone;

  if (fdb_arr == nullptr) {
    out_ = 0;
    SetBits(kPidCalcStateLackFdb, calc_state_);
    return kControllerStateError;
  }

  if (ref_arr == nullptr) {
    SetBits(kPidCalcStateLackRef, calc_state_);
  }

  float out = 0;
  float zero_ffd = 0;
  const float* zero_ffd_ptr = &zero_ffd;

  size_t i = 0;
  for (auto& pid : pids_) {
    // 由于 pid 会记录 前馈值 传入 nullptr ，在此传入一个指向 0 的值
    float node_out = 0;
    float ref = ref_arr == nullptr ? 0 : ref_arr[i];
    pid.calc(&ref, &fdb_arr[i], zero_ffd_ptr, &node_out);
    out += node_out;
    i++;
  }

  if (ffd_arr == nullptr) {
    out_ = out_limit_.calc(out);
  } else {
    out_ = out_limit_.calc(out + ffd_arr[0]);
  }

  if (out_arr != nullptr) {
    out_arr[0] = out_;
  } else {
    SetBits(kPidCalcStateFailedOut, calc_state_);
  }

  SetBits(kPidCalcStateOk, calc_state_);
  return calc_state_ == kPidCalcStateOk
             ? kControllerStateOk
             : kControllerStateError;
}

void MultiNodesPid::setControllerDims(size_t size)
{
  if (type_ == Type::kCascade) {
    ref_dim_ = 1;
  } else if (type_ == Type::kParallel) {
    ref_dim_ = size;
  } else {
    HW_ASSERT(0, "Illegal multi nodes pid type %d", type_);
  }
  ffd_dim_ = size;
  ffd_dim_ = 1;
  out_dim_ = 1;
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace pid
}  // namespace hello_world
