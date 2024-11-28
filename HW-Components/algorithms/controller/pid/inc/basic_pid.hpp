/**
 * @file      basic_pid.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-27
 * @brief
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 *
 * @copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All Rights Reserved.
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 2.1.0 | 2024-01-27 | ZhouShichan | 创建文件 |
 * | 2.1.1 | 2024-02-13 | ZhouShichan | 修复串并级多节点PID计算异常 |
 * | 2.1.2 | 2024-02-20 | ZhouShichan | 修复BasicPid重置逻辑异常 |
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_BASIC_PID_HPP_
#define HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_BASIC_PID_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>
#include <cstdlib>

#include "allocator.hpp"
#include "controller_base.hpp"
#include "pid_opt/pid_opt.hpp"
#include "system.hpp"

namespace hello_world
{
namespace pid
{
/* Exported macro ------------------------------------------------------------*/

enum BasicPidCalcState {
  kPidCalcStateNone = 0u,            ///< 无状态
  kPidCalcStateOk = 1u << 0,         ///< 计算成功
  kPidClacStateOverWait = 1u << 1,   ///< 采样间隔过长
  kPidCalcStateLackRef = 1u << 2,    ///< 缺少参考值
  kPidCalcStateLackFdb = 1u << 3,    ///< 缺少反馈值
  kPidCalcStateFailedOut = 1u << 4,  ///< 传递输出值失败
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
HW_OPTIMIZE_O2_START
// BasicPid 初始化参数
struct __PID_PACKED_PARAMS BasicPidParams : public MemMgr {
  // 基本参数
  bool auto_reset = false;  ///< 是否自动清零
  float kp = 0.0f;          ///< 比例系数
  float ki = 0.0f;          ///< 积分系数
  float kd = 0.0f;          ///< 微分系数
  /*/< 最大控制周期，单位ms，超出表示采样异常, 如果`auto_reset`为true，则会自动清零 */
  uint32_t max_interval_ms = 5;
  // 优化器
  // 输入优化器
  /*/< 设定值平滑优化器 */
  SetpointRamping setpoint_ramping = SetpointRamping(true, -0.1, 0.1, 0.1);
  // 误差计算优化
  DeadBand dead_band = DeadBand(true, -0.001f, 0.001f);  ///< 死区优化器
  PeriodSub period_sub = PeriodSub(false, 0.0f);         ///< 周期最小差值优化器
  // 积分优化
  /*/< 积分抗风up优化器 */
  InteAntiWindup inte_anti_windup = InteAntiWindup(true, -1.0f, 1.0f);
  /*/< 积分变化率优化器 */
  InteChangingRate inte_changing_rate = InteChangingRate(false, 0.0f, 0.001f);
  /*/< 积分分离优化器 */
  InteSeparation inte_separation = InteSeparation(false, 0.0f, 0.0f);
  /*/< 积分梯形优化器 */
  InteTrapezoidal inte_trapezoidal = InteTrapezoidal(true);
  // 差分优化
  /*/< 差分滤波优化器 */
  DiffFilter diff_filter = DiffFilter(true, -0.1f, 0.1f, 0.5f);
  /*/< 差分前馈优化器 */
  DiffPrevious diff_previous = DiffPrevious(true, 0.5f);
  // 输出优化
  OutLimit out_limit = OutLimit(true, -0.1f, 0.1f);  ///< 输出限幅优化器
};

// BasicPid 运行时的动态数据
struct __PID_PACKED_PARAMS BasicPidDatas : public MemMgr {
  float ref = 0.0f;  ///< 参考值
  float fdb = 0.0f;  ///< 反馈值
  float out = 0.0f;  ///< 输出值
  float err = 0.0f;  ///< 错误值

  float last_ref = 0.0f;  ///< 上一次的参考值
  float last_fdb = 0.0f;  ///< 上一次的反馈值
  float last_out = 0.0f;  ///< 上一次的输出值
  float last_err = 0.0f;  ///< 上一次的错误值

  uint32_t last_tick = 0;    ///< 上一次的时刻
  float interval_ms = 1.0f;  ///< 间隔时间，单位 毫秒

  float prop = 0.0f;  ///< 比例项
  float inte = 0.0f;  ///< 积分项
  float diff = 0.0f;  ///< 微分项
  float ffd = 0.0f;   ///< 前馈项

  BasicPidCalcState calc_state = kPidCalcStateNone;  ///< 计算状态
};

class BasicPid : public Controller
{
 public:
  typedef BasicPidParams Params;
  typedef BasicPidDatas Datas;
  typedef BasicPidCalcState CalcState;

  BasicPid(void) : Controller(1, 1, 1, 1) {}
  explicit BasicPid(const Params &params)
      : Controller(1, 1, 1, 1), params_(params) {}
  BasicPid(const BasicPid &) = default;
  BasicPid &operator=(const BasicPid &) = default;
  BasicPid(BasicPid &&) = default;
  BasicPid &operator=(BasicPid &&) = default;

  virtual ~BasicPid(void) = default;

  /**
   * @brief 获取 PID 控制器的参数
   *
   * @return Params& 返回 PID 控制器的参数引用
   * @note 通过引用实现数值传递，可通过该函数修改参数
   */
  Params &params(void) { return params_; }

  /**
   * @brief 获取 PID 控制器的数据
   *
   * @return Datas 返回 PID 控制器的数据
   * @note 通过拷贝构造实现数值传递，不可通过该函数修改数据
   */
  const Datas &datas(void) const { return datas_; }
  /**
   * @brief 计算 PID 控制器的输出
   *
   * @param ref_arr 指向参考值的指针
   * @param fdb_arr 指向反馈值的指针
   * @param ffd_arr 指向前馈值的指针
   * @param out_arr 指向输出值的指针
   * @return BasicPid::State 返回计算状态
   */
  State calc(const float ref_arr[], const float fdb_arr[],
             const float ffd_arr[], float out_arr[]) override;
  /**
   * @brief 重置运行数据
   * @return 始终返回 kControllerStateOk
   */
  State reset(void) override
  {
    datas_.ref = 0.0f;
    datas_.fdb = 0.0f;
    datas_.out = 0.0f;
    datas_.err = 0.0f;
    datas_.last_ref = 0.0f;
    datas_.last_fdb = 0.0f;
    datas_.last_out = 0.0f;
    datas_.last_err = 0.0f;
    // ! 一定要保留上次的时刻，否则会导致采样间隔异常，从而导致控制器持续重置
    datas_.interval_ms = 1.0f;
    datas_.prop = 0.0f;
    datas_.inte = 0.0f;
    datas_.diff = 0.0f;
    datas_.ffd = 0.0f;
    datas_.calc_state = kPidCalcStateNone;
    return kControllerStateOk;
  }

 private:
  Params params_ = Params();  ///< 运行参数
  Datas datas_ = Datas();     ///< 运行数据
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace pid
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_BASIC_PID_HPP_ */
