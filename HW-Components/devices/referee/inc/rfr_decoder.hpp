/** 
 * @file      rfr_decoder.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-02-19
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_DECODER_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_DECODER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "list.hpp"
#include "rfr_pkg/rfr_pkg_core.hpp"

namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

enum RxStatus {
  kRxStatusWaitingHeaderSof = 0u,
  kRxStatusHeader,
  kRxStatusCmdDataTail,
};

union Frame {
  struct __REFEREE_PACKED {
    FrameHeader header;
    CmdId cmd_id;
    uint8_t data[kRefereeMaxFrameLength - sizeof(FrameHeader) - sizeof(CmdId)];
  } decoded;
  uint8_t raw[kRefereeMaxFrameLength];
};

enum RfrDcdResult {
  kRfrDcdUndefinedErr,      ///< 未定义的错误
  kRfrDcdWaitingHeaderSof,  ///< 等待帧头的 SOF 字节
  kRfrDcdGotHeaderSof,      ///< 接收到帧头的 SOF 字节
  kRfrDcdReceivingHeader,   ///< 正在接收帧头
  kRfrDcdFailedCrc8,        ///< 对帧头数据的 CRC8 校验失败
  kRfrDcdGotHeader,         ///< 接收到帧头
  kRfrDcdWaitingTail,       ///< 接收命令 ID 、数据中，等待帧尾
  kRfrDcdFailedCrc16,       ///< 对整帧数据的 CRC16 校验失败
  kRfrDcdOkButNoPkg,        ///< 数据帧正确，但没有找到对应的协议包对象指针进行处理
  kRfrDcdOk,                ///< 数据帧正确且找到对应的协议包对象指针进行处理
};

class RfrDecoder : public MemMgr
{
 public:
  typedef tools::list<ProtocolRxPackage *> ProtocolRxPackageList;

  RfrDecoder(void) = default;
  RfrDecoder(const RfrDecoder &) = default;
  RfrDecoder &operator=(const RfrDecoder &other);
  RfrDecoder(RfrDecoder &&other);
  RfrDecoder &operator=(RfrDecoder &&other);

  ~RfrDecoder(void) = default;
  /**
   * 在串口接收到数据后，调用该函数进行解码
   *
   * 此函数实质是调用 processByte 逐个字节解码
   *
   * @param data_ptr 数据指针
   * @param length 数据长度
   *
   * @return 正确解码的数据包数量
   */
  size_t decodeFrame(const uint8_t *data_ptr, size_t length);

  /**
   * 在串口接收到数据后，调用该函数进行逐个字节解码
   *
   * @param byte 串口接收到的字节
   * @return 解码结果
   */
  RfrDcdResult processByte(uint8_t byte);
  void restartDecodeFrame(void);
  /**
   * 添加一个解包对象
   *
   * @param rx_package_ptr 解包对象指针
   *
   * @attention 该函数会将解包对象指针添加到解包列表中，需要用户自行维护解包对象的生命
   *            周期并从该对象中获取解包数据
   */
  void appendRxPackage(ProtocolRxPackage *rx_package_ptr);

 protected:
  RxStatus rx_status_ = kRxStatusWaitingHeaderSof;
  size_t data_index_ = 0;
  size_t expect_length_ = 0;
  Frame frame_ = {0};
  ProtocolRxPackageList rx_package_list_ = ProtocolRxPackageList();

  bool decodeRxPackage(const CmdId &_cmd_id, const uint8_t *_data_ptr);
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_DECODER_HPP_ */
