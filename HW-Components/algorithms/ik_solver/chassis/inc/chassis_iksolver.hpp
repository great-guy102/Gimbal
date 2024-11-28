/**
 * @file      chassis_iksolver.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2023-12-13
 * @brief     基于约束的轮式移动机器人运动学逆求解器
 * @details   标准轮是各类车轮(转向轮、swedish轮等)的基础，且可以理想化为一个圆形。通
 *            过该圆形施加滚动约束和无侧滑约束，可以求解出满足底盘坐标系下速度指令的各
 *            个车轮的转速。
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
 * | 2.1.0 | 2024-02-11 | ZhouShichan | 对求解方式进行了优化，改变了容器实现方式，更
 *                                      改部分接口函数 |
 * | 2.1.1 | 2024-02-13 | ZhouShichan | 修复固定标准轮、Swedish 轮求解异常；修复
 *                                      Swedish 轮 gamma 值初始化异常； |
 * | 2.1.2 | 2024-02-24 | ZhouShichan | 修复 ChassisIKSolver 的成员函数
 *                                      getThetaVelRefAll getIsNoSideSlipAll 获取
 *                                      数据异常 |
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HWCOMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_CHASSIS_IKSOLVER_HPP_
#define HWCOMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_CHASSIS_IKSOLVER_HPP_

/* Includes ------------------------------------------------------------------*/

#include "allocator.hpp"
#include "base.hpp"
#include "list.hpp"
#include "system.hpp"
#include "wheel/wheel.hpp"

namespace hello_world
{
namespace chassis_ik_solver
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
HW_OPTIMIZE_O2_START

class ChassisIkSolver : public MemMgr
{
 public:
  typedef tools::list<Wheel*> WheelList;

  explicit ChassisIkSolver(const PosVec& control_center = PosVec(0, 0))
      : wheel_list_(WheelList()), control_center_(control_center) {}
  ChassisIkSolver(const ChassisIkSolver&) = default;
  ChassisIkSolver& operator=(const ChassisIkSolver& other);
  ChassisIkSolver(ChassisIkSolver&& other);
  ChassisIkSolver& operator=(ChassisIkSolver&& other);

  ~ChassisIkSolver(void) { clear(); }

  /**
   * @brief 求解轮式移动机器人的底盘逆向运动学问题
   *
   * @param v 输入的移动向量
   * @param theta_vel_fdbs_ptr 传入速度矢量角度反馈数组的指针，默认为 nullptr。对于
   *        转向标准轮、球轮和脚轮，如果为 nullptr 则意味着缺少实时的速度矢量角度，将
   *        采用 0 进行计算。
   * @return IkSolveStatus 返回逆向运动学解的状态。
   */
  IkSolveStatus solve(const MoveVec& v, float* theta_vel_fdbs_ptr);

  /**
   * @brief 求解轮式移动机器人的底盘逆向运动学问题
   *
   * @param v 输入的移动向量
   * @param theta_i2r 移动向量的参考坐标系到底盘坐标系的旋转角度，右手定则判定旋转正方
   *        向，单位：弧度。
   * @param theta_vel_fdbs_ptr 传入速度矢量角度反馈数组的指针，默认为 nullptr。对于
   *        转向标准轮、球轮和脚轮，如果为 nullptr 则意味着缺少实时的速度矢量角度，将
   *        采用 0 进行计算。
   * @return IkSolveStatus 返回逆向运动学解的状态。
   */
  IkSolveStatus solve(
      const MoveVec& v, float theta_i2r, float* theta_vel_fdbs_ptr);
  // 链表操作
  /**
   * @brief 获取轮子链表容器中元素的数量。
   *
   * @return size_t 链表容器中轮子的数量。
   */
  size_t size(void) const { return wheel_list_.size(); }

  /**
   * @brief 向轮子链表容器末尾添加一个新的轮子，类型和参数由传入参数指定。
   *
   * @param wheel_type 要添加的轮子类型。
   * @param params 要添加的轮子的参数。
   * @return bool 如果添加成功则返回 true，否则返回 false。
   */
  bool append(WheelType wheel_type, const WheelParams& params);

  /**  @brief 从链表容器中删除末尾元素并释放对应内存 */
  void erase_tail(void);

  /** @brief 清空链表容器并释放所有对应的内存 */
  void clear(void);

  /**
   * @brief 获取链表容器中指定下标位置的轮子对象的常量指针。
   *
   * 如果下标有效，即在容器大小范围内，则返回指定的轮子对象指针；如果下标无效，则返回
   * nullptr。
   *
   * @param idx 轮子对象在链表中的下标。从 0 开始计数。
   * @return const Wheel* 指向链表中对应下标的轮子对象的常量指针。
   */
  const Wheel* getWheel(size_t idx) const;

  /**
   * @brief 获取链表容器中指定下标位置的轮子对象的常量指针。
   *
   * 如果下标有效，即在容器大小范围内，则返回指定的轮子对象指针；如果下标无效，则返回
   * nullptr。
   *
   * @param idx 轮子对象在链表中的下标。从 0 开始计数。
   * @return Wheel* 指向链表中对应下标的轮子对象的指针。
   */
  Wheel* getWheel(size_t idx);

  // 参数接口

  /**
   * @brief 设置底盘旋转中心的位置。
   *
   * @param x 旋转中心的x坐标。
   * @param y 旋转中心的y坐标。
   */
  void setCenterPos(float x, float y);

  /**
   * @brief 获取底盘旋转中心的位置。
   *
   * @return const PosVec& 底盘旋转中心位置的引用。
   */
  const PosVec& getCenterPos(void) const { return control_center_; }

  /**
   * @brief 获取指定索引对应的逆解算结果。
   *
   * @param idx 逆解算结果的索引。
   * @return const IkSolveRes& 指定索引对应的逆解算结果的引用。
   */
  const IkSolveRes& getIkSolveRes(size_t idx) const;

  /**
   * @brief 获取指定轮子是否无侧滑的状态。
   *
   * @param idx 轮子的索引。
   * @return bool 指定轮子是否无侧滑。
   */
  bool getIsNoSideSlip(size_t idx) const;

  /**
   * @brief 获取指定轮子的旋转速度。
   *
   * @param idx 轮子的索引。
   * @return float 指定轮子的旋转速度。
   */
  float getRotSpd(size_t idx) const;

  /**
   * @brief 获取指定轮子的速度矢量角。
   *
   * @param idx 轮子的索引。
   * @return float 指定轮子的速度矢量角。
   */
  float getThetaVelRef(size_t idx) const;

  /**
   * @brief 获取所有轮子的逆解算结果。
   *
   * @param[out] iksolve_ress_ptr 指向逆解算结果数组的指针。
   * @return IkSolveStatus 执行结果状态。
   */
  IkSolveStatus getIkSolveResAll(IkSolveRes* iksolve_ress_ptr) const;

  /**
   * @brief 获取所有轮子的旋转速度。
   *
   * @param[out] rot_spds_ptr 指向旋转速度数组的指针。
   * @return IkSolveStatus 执行结果状态。
   */
  IkSolveStatus getRotSpdAll(float* rot_spds_ptr) const;

  /**
   * @brief 获取所有轮子的速度矢量角。
   *
   * @param[out] theta_vel_refs_ptr 指向速度矢量角数组的指针。
   * @return IkSolveStatus 执行结果状态。
   */
  IkSolveStatus getThetaVelRefAll(float* theta_vel_refs_ptr) const;

  /**
   * @brief 获取所有轮子是否无侧滑的状态。
   *
   * @param[out] is_no_side_slips_ptr 指向无侧滑状态数组的指针。
   * @return IkSolveStatus 执行结果状态。
   */
  IkSolveStatus getIsNoSideSlipAll(bool* is_no_side_slips_ptr) const;

 private:
  WheelList wheel_list_ = WheelList();    ///< 轮子链表
  PosVec control_center_ = PosVec(0, 0);  ///< 底盘旋转中心位置
  MoveVec vel_r_ = MoveVec(0, 0, 0);      ///< 底盘坐标系下的速度
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

HW_OPTIMIZE_O2_END
}  // namespace chassis_ik_solver
}  // namespace hello_world

#endif /* HWCOMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_CHASSIS_IKSOLVER_HPP_ */
