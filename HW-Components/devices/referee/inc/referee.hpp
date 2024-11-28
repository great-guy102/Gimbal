/**
 *******************************************************************************
 * @file      : referee.hpp
 * @brief     : 裁判系统收发管理类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2024-01-25      ZhouShichan     1. 未测试版本
 *  V1.0.0      2024-07-13      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  该类依赖串口接收管理器 UartRxMgr 与串口发送管理器 UartTxMgr，使用前请确保
 *  UartRxMgr 与 UartTxMgr 按要求配置于初始化，其中串口波特率设置为 115200，字长
 *  8 Bits(include Parity)，无校验，停止位 1。
 *  串口接收管理器中 buf_len 设置为 32，max_process_data_len 设置为 32，eof_type 设
 *  置为 EofType::kManual。
 *  串口发送管理器中 buf_len 建议设置不小于
 *  hello_world::referee::kRefereeMaxFrameLength。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_REFEREE_REFEREE_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_REFEREE_HPP_

/* Includes ------------------------------------------------------------------*/
#include "list.hpp"
#include "offline_checker.hpp"
#include "receiver.hpp"
#include "rfr_pkg/rfr_pkg_core.hpp"
#include "transmitter.hpp"

namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

enum class RefereeReceiveStatus {
  kWaitingHeaderSof = 0u,  ///< 等待 SOF
  kWaitingHeaderCplt,      ///< 等待帧头结束
  kWaitingTailCplt,        ///< 等待帧尾结束
};

enum class RefereeReceiveResult {
  kErrUndefined,      ///< 未定义的错误
  kErrNoDataInput,    ///< 未开始接收数据
  kErrFailedCrc8,     ///< CRC8校验失败
  kErrTooLongHeader,  ///< 帧头长度过长
  kErrTooLongData,    ///< 数据长度过长
  kErrFailedCrc16,    ///< CRC16校验失败
  kHandlingWaitSof,   ///< 处理中-等待 SOF
  kHandlingHeader,    ///< 处理中-正在接收帧头数据
  kHandlingWaitTail,  ///< 处理中-等待帧尾
  kOkWithoutPkg,      ///< 校验通过，但无对应解包
  kOkWithPkg,         ///< 校验通过，解包成功
};

union RefereeCacheFrame {
  struct __REFEREE_PACKED Msg {
    FrameHeader header;
    CmdId cmd_id;
    uint8_t data[kRefereeMaxFrameLength - sizeof(FrameHeader) - sizeof(CmdId)];
  } msg;
  static_assert(sizeof(Msg) == kRefereeMaxFrameLength);
  uint8_t raw[kRefereeMaxFrameLength];
};

class Referee : public comm::Receiver, public comm::Transmitter
{
 public:
  typedef RefereeReceiveStatus RxStatus;
  typedef RefereeReceiveResult RxResult;
  typedef RefereeCacheFrame CacheFrame;
  typedef ProtocolRxPackage RxPkg;
  typedef ProtocolTxPackage TxPkg;

  typedef tools::list<RxPkg *> RxPkgList;

  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  Referee(void) = default;
  /**
   * @brief       构造函数
   * @param        offline_tick_thres: 掉线阈值，为零则自动计算，
   * 使用最小包间隔的3倍计算，单位：ms
   * @retval       None
   * @note        None
   */
  Referee(uint32_t offline_tick_thres)
      : offline_tick_thres_(offline_tick_thres), oc_(offline_tick_thres) {}
  Referee(const Referee &) = default;
  Referee &operator=(const Referee &other);
  Referee(Referee &&other);
  Referee &operator=(Referee &&other);

  virtual ~Referee(void) = default;

  /* 配置方法 */

  /**
   * @brief       初始化，使用默认构造函数后请务必调用此函数
   * @param        offline_tick_thres: 掉线阈值，为零则自动计算，
   * 使用最小包间隔的3倍计算，单位：ms
   * @retval       None
   * @note        None
   */
  void init(uint32_t offline_tick_thres)
  {
    offline_tick_thres_ = offline_tick_thres;
    oc_.init(offline_tick_thres);
  }

  /* 重载方法 */

  virtual uint32_t rxId(void) const override { return 0u; }

  /**
   * @brief       裁判系统数据解包
   * @param        len: 数据长度
   * @param        data: 接收到的数据
   * @retval       解包成功返回true，否则返回false
   * @note        使用前需先调用appendRxPkg注册解包对象
   */
  virtual bool decode(size_t len, const uint8_t *data) override;

  virtual bool isUpdate(void) const override { return is_update_; }

  virtual void clearUpdateFlag(void) override { is_update_ = false; }

  virtual void registerUpdateCallback(pUpdateCallback cb) override
  {
    update_cb_ = cb;
  }

  virtual uint32_t txId(void) const override { return 0u; }

  /**
   * @brief       裁判系统数据编码
   * @param        len: 需传入缓冲区长度，传出编码后的数据长度
   * @param        data: 缓冲区指针
   * @retval       编码成功返回true，否则返回false
   * @note        每次使用前需先调用setTxPkg设置发送数据包
   */
  virtual bool encode(size_t &len, uint8_t *data) override;

  virtual void txSuccessCb(void) override
  {
    tx_success_cnt_++;
    tx_pkg_ready_ = false;
  }

  /* 功能性方法 */

  /**
   * @brief       重置解包
   * @retval       None
   * @note        None
   */
  void resetDecode(void);

  /**
   * @brief       添加一个解包对象
   * @param        rx_pkg_ptr: 解包对象指针，在运行期间不能被释放，
   * 建议为静态变量或全局变量
   * @retval       None
   * @note        None
   */
  void appendRxPkg(RxPkg *rx_pkg_ptr);

  /**
   * @brief       清除解包对象列表
   * @retval       None
   * @note        不建议在运行期间调用
   */
  void clearRxPkgList(void);

  /**
   * @brief       删除一个解包对象
   * @param        rx_pkg_ptr: 解包对象指针
   * @retval       None
   * @note        不建议在运行期间调用
   */
  void eraseRxPkg(RxPkg *rx_pkg_ptr);

  /**
   * @brief       设置发送数据包
   * @param        tx_pkg_ptr: 发送数据包指针，在运行期间不能被释放，
   * 建议为静态变量或全局变量
   * @retval       设置成功返回true，否则返回false
   * @note        只有当发送数据包的发送间隔满足要求且上一个包发送完毕后设置才可能成功，
   * 且设置后只会发送一次，下次发送需要重新设置
   */
  bool setTxPkg(ProtocolTxPackage *tx_pkg_ptr);

  /* 数据修改与获取 */

  RxStatus getRxStatus(void) const { return rx_status_; }

  RxResult getRxResult(void) const { return rx_result_; }

  const CacheFrame &getRxFrame(void) const { return rx_frame_; }

  /**
   * @brief       获取解包成功的数量
   * @retval       解包成功的数量
   * @note        指调用一次decode函数中通过校验并成功解包的数量，
   * 每次调用decode都会覆盖之前的值
   */
  uint32_t getNumRxPkgs(void) const { return rx_n_pkgs_; }

  bool isRxErr(void) const
  {
    /* 根据错误出现的可能，从大到小排序 */
    return rx_result_ == RxResult::kErrFailedCrc8 ||
           rx_result_ == RxResult::kErrFailedCrc16 ||
           rx_result_ == RxResult::kErrTooLongData ||
           rx_result_ == RxResult::kErrUndefined ||
           rx_result_ == RxResult::kErrNoDataInput ||
           rx_result_ == RxResult::kErrTooLongHeader;
  }

  bool isRxOk(void) const
  {
    return rx_result_ == RxResult::kOkWithPkg ||
           rx_result_ == RxResult::kOkWithoutPkg;
  }

  bool isRxHandling(void) const
  {
    return rx_result_ == RxResult::kHandlingWaitTail ||
           rx_result_ == RxResult::kHandlingHeader ||
           rx_result_ == RxResult::kHandlingWaitSof;
  }

  bool isOffline(void) { return oc_.isOffline(); }

 private:
  /* 功能性方法 */

  /**
   * @brief       处理一个接收到的字节
   * @param        byte: 接收到的字节
   * @retval       处理结果
   *   @arg        RxResult::kErrUndefined: 未定义错误
   *   @arg        RxResult::kHandlingWaitSof: 等待帧头
   *   @arg        RxResult::kHandlingHeader: 正在处理帧头
   *   @arg        RxResult::kHandlingWaitTail: 正在等待帧尾
   *   @arg        RxResult::kErrFailedCrc8: CRC8校验失败
   *   @arg        RxResult::kErrTooLongHeader: 帧头过长
   *   @arg        RxResult::kErrTooLongData: 数据过长
   *   @arg        RxResult::kOkWithPkg: 解包成功
   *   @arg        RxResult::kOkWithoutPkg: 解包失败
   *   @arg        RxResult::kErrFailedCrc16: CRC16校验失败
   * @note        None
   */
  RxResult processByte(uint8_t byte);

  /**
   * @brief       解包一个数据包
   * @param        cmd_id: 命令ID
   * @param        data_ptr: 数据指针
   * @retval       解包成功返回true，否则返回false
   * @note        None
   */
  bool decodeRxPackage(const CmdId &cmd_id, const uint8_t *data_ptr);

  /* rx */

  CacheFrame rx_frame_ = {0};  ///< 接收到的帧数据

  RxPkgList rx_pkg_list_ = RxPkgList();  ///< 解包对象列表

  RxStatus rx_status_ = RxStatus::kWaitingHeaderSof;  ///< 接收状态
  RxResult rx_result_ = RxResult::kErrNoDataInput;    ///< 解包结果
  bool is_update_ = false;
  pUpdateCallback update_cb_ = nullptr;

  size_t rx_data_index_ = 0;     ///< 接收到的数据索引
  size_t rx_expect_length_ = 0;  ///< 期望接收的数据长度
  uint32_t rx_n_pkgs_ = 0;       ///< 通过 decode 解包成功的数量
  uint8_t rx_err_cnt_ = 0;       ///< 错误计数

  /*/< 掉线阈值，为零则自动计算，使用最小包间隔的3倍计算，单位：ms */
  uint32_t offline_tick_thres_ = 0;
  OfflineChecker oc_ = OfflineChecker(1000);  ///< 离线检查器

  /* tx */

  CacheFrame tx_frame_ = {0};

  bool tx_pkg_ready_ = false;  ///< 是否有待发送的数据
  size_t tx_frame_len_ = 0;    ///< 待发送数据长度

  uint32_t encode_success_cnt_ = 0;
  uint32_t encode_fail_cnt_ = 0;
  uint32_t tx_success_cnt_ = 0;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_REFEREE_HPP_ */
