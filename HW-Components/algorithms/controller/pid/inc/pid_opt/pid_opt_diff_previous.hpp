/**
 * @file      pid_opt_diff_previous.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-27
 * @brief     差分先行优化器的实现
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 *
 * @copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All Rights Reserved.
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2024-01-27 | ZhouShichan | 创建文件 |
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_DIFF_PREVIOUS_HPP_
#define HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_DIFF_PREVIOUS_HPP_

/* Includes ------------------------------------------------------------------*/
#include "pid_opt_core.hpp"
#include "system.hpp"

namespace hello_world
{
namespace pid
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
HW_OPTIMIZE_O2_START
/**
 * @class DiffPrevious
 * @brief 微分先行优化器
 *
 * 为避免由于给定值频繁升降（尤其是阶跃）而引起的系统振荡，可采用只对输出(或反馈值)微分
 * 的优化算法。
 * 这样，在改变给定值时，微分项输出仅与被控量的变化相关，而这种变化通常是比较缓和的，从
 * 而能够明显改善系统的动态特性。
 * 更进一步，将对输入误差微分和对输出(反馈)微分采用加权求和的方式结合起来。
 * - 具体计算公式见`calc`
 * - 参数合法性检查见`isLegal`
 */
class __PID_PACKED_PARAMS DiffPrevious : public PidOptimizer
{
 public:
  DiffPrevious(void) = default;
  /** 构造函数 @see setParams */
  DiffPrevious(bool is_enabled, float weight)
  {
    setParams(is_enabled, weight);
  }
  DiffPrevious(const DiffPrevious &) = default;
  DiffPrevious &operator=(const DiffPrevious &) = default;
  DiffPrevious(DiffPrevious &&) = default;
  DiffPrevious &operator=(DiffPrevious &&) = default;

  virtual ~DiffPrevious(void) = default;

  /** 设置反馈值微分的权重值 @see setParams */
  State setWeight(float weight) { return setParams(enabled_, weight); }

  /** 获取反馈值微分的权重值 */
  float getWeight(void) const { return weight_; }

  /** 当且仅当权重值在[0,1]上时，参数合法 */
  bool isLegal(void) { return 0 <= weight_ && weight_ <= 1; }

  /**
   * @brief 设置微分先行优化参数
   * @param is_enabled 是否启用微分先行优化
   * @param weight 对反馈值微分的权重值
   * @retval kPidOptStateOk 参数合法，设置成功
   * @retval kPidOptStateIllegalParams 参数非法，自动修改后设置成功
   */
  State setParams(bool is_enabled, float weight)
  {
    enabled_ = is_enabled;
    weight_ = weight;
    return handleIllegalParams() ? kPidOptStateIllegalParams : kPidOptStateOk;
  }

  /**
   * @brief 检查和处理非法参数
   *
   * 如果权重小于0，设置权重为0。如果权重大于1，设置权重为1。保证权重值在[0,1]上。
   * @retval true 参数非法，已经自动处理
   * @retval false 参数合法
   */
  bool handleIllegalParams(void)
  {
    bool with_illegal_params = false;
    if (weight_ < 0) {
      weight_ = 0;
      with_illegal_params = true;
    } else if (weight_ > 1) {
      weight_ = 1;
      with_illegal_params = true;
    }
    return with_illegal_params;
  }

  /**
   * @brief 计算微分先行优化后的输出值
   *
   * 当且仅当优化器使能、参数合法时，将误差微分值和反馈值微分值按照权重值加权求和，作为
   * 微分项的输出；否则直接返回误差微分值。
   * @details 具体的计算公式为：
   * \f[\frac{u_d(k)}{T(k)K_d}=\begin{cases}
   * w*[y(k)-y(k-1)]+(1-w)*[e(k)-e(k=1)]  & ,  w \in (0,1]   \\
   * e(k)-e(k=1)  & ,others
   * \end{cases}
   * \f]，
   *  其中 \f$\epsilon_l\f$ 为参数中的下限值，
   * \f$\epsilon_u\f$为参数中的上限值，
   * \f$w\f$为参数中的权重值
   * @param fdb 本次采样的反馈值
   * @param last_fdb 上次采样的反馈值
   * @param err 本次采样的误差值
   * @param last_err 上次采样的误差值
   * @return 本次采样的微分先行优化后的输出值
   */
  float calc(float fdb, float last_fdb, float err, float last_err)
  {
    if (isEnabled() && isLegal()) {
      return ((1 - weight_) * (err - last_err) + weight_ * (fdb - last_fdb));
    } else {
      return err - last_err;
    }
  }

 private:
  float weight_ = 0.0f;  ///< 对输出微分的权重值
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace pid
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_CONTROLLER_PID_PID_OPT_PID_OPT_DIFF_PREVIOUS_HPP_ */
