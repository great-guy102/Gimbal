# HW-Components-Referee
裁判系统串口协议子模块，需要结合 httpsgithub.comZJU-HelloWorldHW-Components 分支 cpp24 使用

## 快速开始

### 解包

实例化一个 `RfrDecoder` 对象，添加所需的数据包指针，在串口回调函数中传入待解包的字节流，即可得到解包结果。

注意:

 1. 添加的数据包指针必须是 `ProtocolRxPackage` 及其子类的指针，且其指向的对象生命周期必须大于等于 `RfrDecoder` 对象。
 2. 解包时，`RfrDecoder` 对象会自动跳过 `0x5A` 之前的无效字节，并在 CRC 校验失败时自动退出解包流程，不再继续解包，即使后续字节流中还有有效数据。
 3. 解包时，每个 `RfrDecoder` 对象都含有一个 `uint8_t` 类型的缓冲区，用于存储当前帧的字节流。如果中断处理函数出现同级嵌套，即多次串口数据短时间内同时到达触发中断、且上一次解包未完成，则可能导致解包结果错误或解码失败。

```c++
#include "referee.hpp"
#include "rfr_official_pkgs.hpp"

namespace rfr = hello_world::referee;

rfr::RfrDecoder rfr_decoder;

// pkgs
rfr::RobotPerformancePackage robot_perf_pkg;
rfr::RobotPowerHeatPackage robot_power_heat_pkg;
rfr::RobotShooterPackage robot_shooter_pkg;
// more pkgs...

void RfrDecoderInit() {
    // add pkgs
    rfr_decoder.appendRxPackage(&robot_perf_pkg);
    rfr_decoder.appendRxPackage(&robot_power_heat_pkg);
    rfr_decoder.appendRxPackage(&robot_shooter_pkg);
    // more pkgs...
}

// not valid callback function, just for example
void UartCallbackWithDataArray(uint8_t* data_ptr, uint16_t len) {
    // method 1: use pointer to data array
    // ! 请注意，当多个包在短时间内同时达到串口，但包与包之间的间隔太短，并不能触发 DMA 的空闲中断
    // ！而当前版本中的 decodeFrame 函数在成功解包一次之后就会跳过之后的字节流，哪怕后续字节流中还有有效数据
    // ! 因此，哪怕是在 DMA 空闲中断中解包，也建议使用方法 2 解包，即逐字节解包
    rfr_decoder.decodeFrame(data_ptr, len);
    // method 2: input byte one by one [recommended]
    for (uint16_t i = 0; i < len; i++) {
        rfr_decoder.decodeByte(data_ptr[i]);
    } 
}
```

### 编码

实例化一个 `RfrEncoder` 对象，添加所需的数据包指针和编码结果指针，即可得到编码结果。

注意:

 1. 添加的数据包指针必须是 `ProtocolTxPackage` 及其子类的指针，且其指向的对象生命周期必须大于等于调用函数 `encodeFrame` 的生命周期。
 2. 编码时，`RfrEncoder` 对象会在传入非法数据、 CRC8 校验码设置失败、CRC16 校验码设置失败、传入数据包数据编码失败等情况时，返回 `false`，但并不会对 `frame_ptr` 和 `frame_len` 进行修改；只有当所有编码流程都成功完成时，才会将编码数据从内部缓冲区拷贝到 `frame_ptr` 指向的内存中，并返回 `true`。因此需要自行检查返回值。
 3. 编码时，每个 `RfrEncoder` 对象都含有一个 `uint8_t` 类型的缓冲区，用于存储当前帧的字节流。如果中断处理函数出现同级嵌套，即上一次编码尚未完成，但有又一次编码请求，则可能导致编码结果错误或者编码失败。

```c++
#include "referee.hpp"
#include "rfr_official_pkgs.hpp"

namespace rfr = hello_world::referee;

rfr::RfrEncoder rfr_encoder;

uint8_t tx_data[kRefereeMaxFrameLength]={0};
size_t tx_len = 0;

rfr::InterGraphic7Package inter_graphic7_pkg;

// pkgs
void SendPkgExample() {
    // encode pkgs
    rfr_encoder.encodeFrame(&inter_graphic7_pkg, tx_data, &tx_len);
    // send data(not valid function, just for example)
    uart_send_data(tx_data, tx_len);
};

```

## 邪道用法

### 编译宏定义缓冲区大小

在 `rfr_pkg_core.hpp` 中，存在常数 `const size_t kRefereeMaxFrameLength`， 可被宏 `RFR_MAX_FRAME_LENGTH` 定义覆盖，以改变缓冲区大小。默认为 135，即最大帧长为 135 字节。除此之外，`RFR_MAX_FRAME_LENGTH` 不超过 135 字节不会覆盖默认值。此常数会改变 `RfrDecoder` 和 `RfrEncoder` 对象的缓冲区大小。


## 数据包汇总

### 官方数据包

#### 0x0001 比赛状态数据