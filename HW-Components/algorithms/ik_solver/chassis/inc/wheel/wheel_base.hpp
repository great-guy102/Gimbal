/**
 *******************************************************************************
 * @file      : wheel_base.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_WHEEL_BASE_HPP_
#define HW_COMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_WHEEL_BASE_HPP_

/* Includes ------------------------------------------------------------------*/
#include "base.hpp"
#include "system.hpp"
#include "wheel_vec.hpp"

namespace hello_world
{
namespace chassis_ik_solver
{
/* Exported macro ------------------------------------------------------------*/
// 逆解结果
enum IkSolveStatus {
  kIkSolveStatusOk = 0u,                   ///< 逆解求解成功
  kIkSolveStatusLeakVelAngFdb = 1u << 1,   ///< 缺少速度角反馈
  kIkSolveStatusRadiusTooSmall = 1u << 2,  ///< 车轮半径过小
  kIkSolveStatusLengthTooSmall = 1u << 3,  ///< 车轮中心到转动中心的距离过小
  kIkSolveStatusFailReturn = 1u << 4,      ///< 传递结果失败
};
/**
 * @brief 车轮类型
 *
 * 这个枚举定义了所有可能的车轮类型，包括：
 *
 * - 固定标准轮
 * - 转向标准轮
 * - 脚轮(解算麻烦又用不到，暂时不实现)
 * - 瑞典轮
 *  - 麦克纳姆轮，一种特殊的瑞典轮，且 gamma = pi / 4
 *  - 全向轮，一种特殊的瑞典轮，且 gamma = 0
 * - 球轮
 */
enum class WheelType {
  kNotImplemented = 0u,  ///< 未实现
  kFixedStandard,        ///< 固定标准轮
  kSteeredStandard,      ///< 转向标准轮
  kCastor,               ///< 脚轮
  kSwedish,              ///< 瑞典轮
  kMecanum,              ///< 麦克纳姆轮(瑞典轮的一种)
  kOmni,                 ///< 全向轮(瑞典轮的一种)
  kSpherical,            ///< 球轮
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
HW_OPTIMIZE_O2_START

/**
 * @note 如何确定车轮转动矢量：
 * @note 1. 车轮正向旋转时，地面接触点的线速度方向的反方向；
 * @note 2. 车轮法向量与底盘坐标系Z轴叉乘得到的向量即为车轮转动矢量。
 * @attention 对于转向标准轮、球轮、脚轮的逆解需要实时更新进行优化。
 * @attention 对于麦克纳姆轮，由于代码中 gamma 是固定的 pi/4，因此需要特别确定其转向角
 *            度使其 gamma 值在车辆上满足 pi/4。
 */
struct WheelParams {
  uint32_t opt_mask = 0u;      ///< 优化选项掩码
  float theta_vel_fdb = 0.0f;  ///< 反馈的车轮转向角度
  float d_castor = 0.0f;       ///< 车轮中心到车轮垂直转轴的距离，单位：m
  /*/< 车轮转轴与车轮垂直转轴之间的最小夹角，单位：rad */
  float gamma = 0.0f;
  float radius = 0.0f;               ///< 车轮半径，单位：m
  PosVec wheel_pos = PosVec(0, 0);   ///< 车轮垂直转轴在底盘坐标系下的位置，单位：m
  PosVec* center_pos_ptr = nullptr;  ///< 底盘旋转中心位置，单位：m
};

struct IkSolveRes {
  bool is_no_side_slip = false;  ///< 此次运动指令是否满足无侧滑约束
  float rot_spt = 0.0f;          ///< 逆解的车轮转动速度，单位：rad/s
  float theta_vel_ref = 0.0f;    ///< 逆解的车轮转向角度，单位：rad
};

class Wheel : public MemMgr
{
 public:
  enum OptMask {
    kOptMaskNone = 0u,
  };

  explicit Wheel(const WheelParams& params)
  {
    setParams(params);
  }
  Wheel(const Wheel&) = default;
  Wheel& operator=(const Wheel&) = default;
  Wheel(Wheel&&) = default;
  Wheel& operator=(Wheel&&) = default;

  virtual ~Wheel(void) = default;

  virtual IkSolveStatus ikSolve(
      const MoveVec& v, IkSolveRes* res_ptr,
      const float* theta_vel_fdb_ptr = nullptr) = 0;

  // 逆解结果获取接口
  /**
   * @brief 获取逆运动学求解结果的引用。
   *
   * @return const IkSolveRes& 逆运动学求解结果的常量引用。
   */
  const IkSolveRes& getIkSolveRes(void) const { return iksolve_res_; }

  /**
   * @brief 获取指示是否无侧滑条件的标志。
   *
   * @return bool 如果没有侧滑发生，则返回true，否则返回false。
   */
  bool getIsNoSideSlip(void) const { return iksolve_res_.is_no_side_slip; }

  /**
   * @brief 获取旋转速度值。
   *
   * @return float 返回旋转速度(rotational speed)的值。
   */
  float getRotSpd(void) const { return iksolve_res_.rot_spt; }

  /**
   * @brief 获取参考的角速度值。
   *
   * @return float 返回参考的角速度(theta velocity reference)值。
   */
  float getThetaVelRef(void) const { return iksolve_res_.theta_vel_ref; }

  // 参数设置接口
  /**
   * @brief 设置轮子参数。
   *
   * @param params 轮子参数的结构体。
   */
  void setParams(const WheelParams& params)
  {
    setOptMask(params.opt_mask);
    setThetaVelFdb(params.theta_vel_fdb);
    setDCaster(params.d_castor);
    setRadius(params.radius);
    setWheelPos(params.wheel_pos);
    setCenterPos(params.center_pos_ptr);
    setGamma(params.gamma);
    setPosCallback();
  }

  /**
   * @brief 设置优化掩码。
   *
   * @param opt_mask 优化选项掩码。
   */
  void setOptMask(uint32_t opt_mask) { params_.opt_mask = opt_mask; }

  /**
   * @brief 设置反馈的速度矢量角。
   *
   * @param theta_vel 反馈的速度矢量角。
   */
  void setThetaVelFdb(float theta_vel) { params_.theta_vel_fdb = theta_vel; }

  /**
   * @brief 设置轮子半径，并确保半径不小于一个预设的最小值。
   *
   * @param radius 要设置的轮子半径。
   */
  void setRadius(float radius)
  {
    params_.radius = radius < 0.01 ? 0.01 : radius;
  }

  /**
   * @brief 设置轮子位置。
   *
   * @param pos 轮子位置的向量。
   */
  void setWheelPos(const PosVec& pos)
  {
    params_.wheel_pos = pos;
    setPosCallback();
  }

  /**
   * @brief 设置底盘转轴的位置指针。
   *
   * @param pos_ptr 指向底盘转轴的位置的指针。
   */
  void setCenterPos(PosVec* pos_ptr)
  {
    params_.center_pos_ptr = pos_ptr;
    setPosCallback();
  }

  /**
   * @brief 设置Gamma值，虚函数，可能在子类中实现具体功能。
   *
   * @param gamma Gamma值。
   */
  virtual void setGamma(float gamma) { params_.gamma = 0; }

  /**
   * @brief 设置轮子垂直旋转轴和轮子中心的距离，虚函数，可能在子类中实现具体功能。
   *
   * @param d 轮子垂直旋转轴和轮子中心的距离。
   */
  virtual void setDCaster(float d) { params_.d_castor = 0; }

  // 参数获取接口
  /**
   * @brief 获取当前设置的轮子参数。
   * @return WheelParams 已设置的轮子参数的结构体。
   */
  const WheelParams& getParams(void) const { return params_; }

  /**
   * @brief 获取设置的优化掩码值。
   * @return uint32_t 当前设置的优化掩码。
   */
  uint32_t getOptMask(void) const { return params_.opt_mask; }

  /**
   * @brief 获取设置的反馈的速度矢量角。
   *
   * @return float 当前设置的反馈的速度矢量角。
   */
  float getThetaVelFdb(void) const { return params_.theta_vel_fdb; }

  /**
   * @brief 获取轮子垂直旋转轴和轮子中心的距离。
   *
   * @return float 当前设置的轮子垂直旋转轴和轮子中心的距离。
   */
  float getDCaster(void) const { return params_.d_castor; }

  /**
   * @brief 获取当前 alpha 值。
   *
   * @return float 当前 alpha 值。
   */
  float getAlpha(void) const { return alpha_; }

  /**
   * @brief 根据传入的速度矢量角计算Beta值。
   *
   * @param theta_vel 速度矢量角。
   * @return float 计算出的Beta值。
   */
  float getBeta(float theta_vel) const { return theta_vel - alpha_ + M_PI_2; }

  /**
   * @brief 获取Gamma值。
   *
   * @return float 当前设置的Gamma值。
   */
  float getGamma(void) const { return params_.gamma; }

  /**
   * @brief 获取轮子半径。
   *
   * @return float 当前设置的轮子半径。
   */
  float getRadius(void) const { return params_.radius; }

  /**
   * @brief 获取车轮垂直旋转轴到底盘旋转中心的距离。
   *
   * @return float 车轮垂直旋转轴到底盘旋转中心的距离。
   */
  float getL(void) const { return l_; }

  /**
   * @brief 获取轮子位置。
   *
   * @return PosVec 当前设置的轮子位置。
   */
  const PosVec& getWheelPos(void) const { return params_.wheel_pos; }

  /**
   * @brief 获取指向底盘转轴位置的指针。
   *
   * @return const PosVec* 指向当前设置的底盘转轴位置的指针。
   */
  const PosVec* getCenterPos(void) const { return params_.center_pos_ptr; }

 protected:
  void setPosCallback(void)
  {
    PosVec dist = params_.wheel_pos;
    if (params_.center_pos_ptr != nullptr) {
      dist -= *params_.center_pos_ptr;
    }
    alpha_ = dist.ang();
    l_ = dist.norm();
  }

  float alpha_ = 0;  ///< x 轴正方向与底盘旋转中心到车轮垂直旋转轴的有向线段的夹角
  float l_ = 0;      ///< 底盘旋转中心到车轮垂直旋转轴的有向线段长度
  IkSolveRes iksolve_res_ = {
      .is_no_side_slip = false, .rot_spt = 0, .theta_vel_ref = 0};
  WheelParams params_ = {
      .opt_mask = 0,
      .theta_vel_fdb = 0,
      .d_castor = 0,
      .gamma = 0,
      .radius = 0,
      .wheel_pos = PosVec(0, 0),
      .center_pos_ptr = nullptr,
  };
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace chassis_ik_solver
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_WHEEL_WHEEL_BASE_HPP_ */
