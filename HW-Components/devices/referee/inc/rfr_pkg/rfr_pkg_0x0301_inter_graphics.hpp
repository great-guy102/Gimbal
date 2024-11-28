/**
 * @file      rfr_pkg_0x0301_inter_graphics.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-25
 * @brief
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 *
 * @copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All
 * Rights Reserved.
 *
 * @attention
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2024-02-18 | ZhouShichan | 首次完成 |
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0301_INTER_GRAPHICS_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0301_INTER_GRAPHICS_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstring>

#include "rfr_pkg_0x0301_inter_among_robots.hpp"
#include "system.hpp"

namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/

enum DeleteOperation : uint8_t {
  kDeleteNone = 0x00,   ///< 空操作
  kDeleteLayer = 0x01,  ///< 删除图层
  kDeleteAll = 0x02,    ///< 删除所有图形
};

enum GraphicLayer : uint8_t {
  kGraphicLayer0 = 0x00,  ///< 图层 0
  kGraphicLayer1 = 0x01,  ///< 图层 1
  kGraphicLayer2 = 0x02,  ///< 图层 2
  kGraphicLayer3 = 0x03,  ///< 图层 3
  kGraphicLayer4 = 0x04,  ///< 图层 4
  kGraphicLayer5 = 0x05,  ///< 图层 5
  kGraphicLayer6 = 0x06,  ///< 图层 6
  kGraphicLayer7 = 0x07,  ///< 图层 7
  kGraphicLayer8 = 0x08,  ///< 图层 8
  kGraphicLayer9 = 0x09,  ///< 图层 9
};

enum GraphicOperation : uint8_t {
  kGraphicNone = 0x00,    ///< 空操作
  kGraphicAdd = 0x01,     ///< 添加图形
  kGraphicModify = 0x02,  ///< 修改图形
  kGraphicDelete = 0x03,  ///< 删除图形
};

enum GraphicType : uint8_t {
  kGraphicStraightLine = 0x00,  ///< 直线
  kGrahicRectangle = 0x01,      ///< 矩形
  kGraphicCircle = 0x02,        ///< 圆形
  kGraphicEllipse = 0x03,       ///< 椭圆
  kGraphicArc = 0x04,           ///< 弧线
  kGraphicFloating = 0x05,      ///< 浮点数
  kGraphicInteger = 0x06,       ///< 整数
  kGraphicString = 0x07,        ///< 字符串
  kGraphicUnKnown,              ///< 未知
};

enum GraphicColor : uint8_t {
  kColorRedOrBlue = 0x00,  ///< 主色，红/蓝（己方颜色）
  kColorYellow = 0x01,     ///< 黄色
  kColorGreen = 0x02,      ///< 绿色
  kColorOrange = 0x03,     ///< 橙色
  kColorPurple = 0x04,     ///< 紫色
  kColorPink = 0x05,       ///< 粉色
  kColorCyan = 0x06,       ///< 青色
  kColorBlack = 0x07,      ///< 黑色
  kColorWhite = 0x08,      ///< 白色
};

const uint16_t kMaxPosX = 1920u;
const uint16_t kMaxPosY = 1080u;
const size_t kMaxStringLength = 30u;

enum GraphicDataMask : uint32_t {
  kMaskOperateType = (0x01 << 3) - 1,
  kMaskFigureType = (0x01 << 3) - 1,
  kMaskLayer = (0x01 << 4) - 1,
  kMaskColor = (0x01 << 4) - 1,
  kMaskDetailsA = (0x01 << 9) - 1,
  kMaskDetailsB = (0x01 << 9) - 1,
  kMaskLineWidth = (0x01 << 10) - 1,
  kMaskStartX = (0x01 << 11) - 1,
  kMaskStartY = (0x01 << 11) - 1,
  kMaskDetailsC = (0x01 << 10) - 1,
  kMaskDetailsD = (0x01 << 11) - 1,
  kMaskDetailsE = (0x01 << 11) - 1,
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef uint16_t Pixel;  ///< 像素
typedef uint16_t Ang;    ///< 角度

struct __REFEREE_PACKED GraphicDeleteData {
  uint8_t delete_operation;
  uint8_t layer;
};

struct __REFEREE_PACKED GraphicData {
  uint8_t figure_name[3];
  uint32_t operate_tpye : 3;
  uint32_t figure_tpye : 3;
  uint32_t layer : 4;
  uint32_t color : 4;
  uint32_t details_a : 9;
  uint32_t details_b : 9;
  uint32_t line_width : 10;
  uint32_t start_x : 11;
  uint32_t start_y : 11;
  uint32_t details_c : 10;
  uint32_t details_d : 11;
  uint32_t details_e : 11;
};

class Graphic : public MemMgr
{
 public:
  typedef GraphicOperation Operation;
  typedef GraphicLayer Layer;
  typedef GraphicType Type;
  typedef GraphicColor Color;
  typedef GraphicData Data;

  Graphic(void) = default;

  Graphic(const uint8_t name[3], Operation operation, Layer layer, Color color)
  {
    setName(name);
    setOperation(operation);
    setLayer(layer);
    setColor(color);
  }

  void setName(const uint8_t name[3]) { memcpy(data_.figure_name, name, 3); }
  void setOperation(Operation operation)
  {
    data_.operate_tpye = (uint32_t)operation & (uint32_t)kMaskOperateType;
  }
  void setLayer(Layer layer)
  {
    data_.layer = (uint32_t)layer & (uint32_t)kMaskLayer;
  }
  void setColor(Color color)
  {
    data_.color = (uint32_t)color & (uint32_t)kMaskColor;
  }
  void setLineWidth(Pixel line_width)
  {
    data_.line_width = (uint32_t)line_width & (uint32_t)kMaskLineWidth;
  }

  const uint8_t *getName(void) const { return data_.figure_name; }
  Operation getOperation(void) const { return (Operation)data_.operate_tpye; }
  Layer getLayer(void) const { return (Layer)data_.layer; }
  Color getColor(void) const { return (Color)data_.color; }
  Pixel getLineWidth(void) const { return data_.line_width; }

  const Data &getData(void) const { return data_; }

 protected:
  void setGraphicType(const Type &type)
  {
    data_.figure_tpye = (uint32_t)type & (uint32_t)kMaskFigureType;
  }
  Data data_ = {
      .figure_name = {0u, 0u, 0u},
      .operate_tpye = Operation::kGraphicNone,
      .figure_tpye = Type::kGraphicStraightLine,
      .layer = Layer::kGraphicLayer0,
      .color = Color::kColorRedOrBlue,
      .details_a = 0,
      .details_b = 0,
      .line_width = 0,
      .start_x = 0,
      .start_y = 0,
      .details_c = 0,
      .details_d = 0,
      .details_e = 0,
  };
};

class StraightLine : public Graphic
{
 public:
  static constexpr Type kGraphicType = Type::kGraphicStraightLine;  ///< 图形类型

  StraightLine(void) { setGraphicType(kGraphicType); }

  explicit StraightLine(
      const uint8_t name[3], Operation operation, Layer layer, Color color)
      : Graphic(name, operation, layer, color)
  {
    setGraphicType(kGraphicType);
  }

  explicit StraightLine(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel start_x, Pixel start_y, Pixel end_x, Pixel end_y)
      : Graphic(name, operation, layer, color)
  {
    setStartPos(start_x, start_y);
    setEndPos(end_x, end_y);
    setGraphicType(kGraphicType);
    setLineWidth(1);
  }

  explicit StraightLine(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel start_x, Pixel start_y, Pixel end_x, Pixel end_y, Pixel line_width)
      : Graphic(name, operation, layer, color)
  {
    setStartPos(start_x, start_y);
    setEndPos(end_x, end_y);
    setLineWidth(line_width);
    setGraphicType(kGraphicType);
  }

  void setStartPos(Pixel x, Pixel y)
  {
    setStartPosX(x);
    setStartPosY(y);
  }
  void setStartPosX(Pixel x) { data_.start_x = x; }
  void setStartPosY(Pixel y) { data_.start_y = y; }
  void setEndPos(Pixel x, Pixel y)
  {
    setEndPosX(x);
    setEndPosY(y);
  }
  void setEndPosX(Pixel x) { data_.details_d = x & kMaskDetailsD; }
  void setEndPosY(Pixel y) { data_.details_e = y & kMaskDetailsE; }

  Pixel getStartPosX(void) const { return data_.start_x; }
  Pixel getStartPosY(void) const { return data_.start_y; }
  Pixel getEndPosX(void) const { return data_.details_d; }
  Pixel getEndPosY(void) const { return data_.details_e; }
};
class Rectangle : public Graphic
{
 public:
  static constexpr Type kGraphicType = Type::kGrahicRectangle;  ///< 图形类型

  Rectangle(void) { setGraphicType(kGraphicType); }

  explicit Rectangle(
      const uint8_t name[3], Operation operation, Layer layer, Color color)
      : Graphic(name, operation, layer, color)
  {
    setGraphicType(kGraphicType);
  }

  explicit Rectangle(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel start_x, Pixel start_y, Pixel end_x, Pixel end_y)
      : Graphic(name, operation, layer, color)
  {
    setStartPos(start_x, start_y);
    setEndPos(end_x, end_y);
    setGraphicType(kGraphicType);
    setLineWidth(1);
  }

  explicit Rectangle(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel start_x, Pixel start_y, Pixel end_x, Pixel end_y, Pixel line_width)
      : Graphic(name, operation, layer, color)
  {
    setStartPos(start_x, start_y);
    setEndPos(end_x, end_y);
    setLineWidth(line_width);
    setGraphicType(kGraphicType);
  }

  void setStartPos(Pixel x, Pixel y)
  {
    setStartPosX(x);
    setStartPosY(y);
  }
  void setStartPosX(Pixel x) { data_.start_x = x; }
  void setStartPosY(Pixel y) { data_.start_y = y; }
  void setEndPos(Pixel x, Pixel y)
  {
    setEndPosX(x);
    setEndPosY(y);
  }
  void setEndPosX(Pixel x) { data_.details_d = x & kMaskDetailsD; }
  void setEndPosY(Pixel y) { data_.details_e = y & kMaskDetailsE; }

  Pixel getStartPosX(void) const { return data_.start_x; }
  Pixel getStartPosY(void) const { return data_.start_y; }
  Pixel getEndPosX(void) const { return data_.details_d; }
  Pixel getEndPosY(void) const { return data_.details_e; }
};
class Circle : public Graphic
{
 public:
  static constexpr Type kGraphicType = Type::kGraphicCircle;  ///< 图形类型

  Circle(void) { setGraphicType(kGraphicType); }

  explicit Circle(
      const uint8_t name[3], Operation operation, Layer layer, Color color)
      : Graphic(name, operation, layer, color)
  {
    setGraphicType(kGraphicType);
  }

  explicit Circle(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel center_x, Pixel center_y, Pixel radius)
      : Graphic(name, operation, layer, color)
  {
    setCenterPos(center_x, center_y);
    setRadius(radius);
    setGraphicType(kGraphicType);
    setLineWidth(1);
  }

  explicit Circle(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel center_x, Pixel center_y, Pixel radius, Pixel line_width)
      : Graphic(name, operation, layer, color)
  {
    setCenterPos(center_x, center_y);
    setRadius(radius);
    setLineWidth(line_width);
    setGraphicType(kGraphicType);
  }

  void setCenterPos(Pixel x, Pixel y)
  {
    setCenterPosX(x);
    setCenterPosY(y);
  }
  void setCenterPosX(Pixel x) { data_.start_x = x; }
  void setCenterPosY(Pixel y) { data_.start_y = y; }
  void setRadius(Pixel r) { data_.details_c = r & kMaskDetailsC; }

  Pixel getCenterPosX(void) const { return data_.start_x; }
  Pixel getCenterPosY(void) const { return data_.start_y; }
  Pixel getRadius(void) const { return data_.details_c; }
};
class Ellipse : public Graphic
{
 public:
  static constexpr Type kGraphicType = Type::kGraphicEllipse;  ///< 图形类型

  Ellipse(void) { setGraphicType(kGraphicType); }

  explicit Ellipse(
      const uint8_t name[3], Operation operation, Layer layer, Color color)
      : Graphic(name, operation, layer, color)
  {
    setGraphicType(kGraphicType);
  }

  explicit Ellipse(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel center_x, Pixel center_y, Pixel radius_x, Pixel radius_y)
      : Graphic(name, operation, layer, color)
  {
    setCenterPos(center_x, center_y);
    setRadius(radius_x, radius_y);
    setGraphicType(kGraphicType);
    setLineWidth(1);
  }

  explicit Ellipse(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel center_x, Pixel center_y, Pixel radius_x, Pixel radius_y,
      Pixel line_width)
      : Graphic(name, operation, layer, color)
  {
    setCenterPos(center_x, center_y);
    setRadius(radius_x, radius_y);
    setLineWidth(line_width);
    setGraphicType(kGraphicType);
  }

  void setCenterPosX(Pixel x) { data_.start_x = x & kMaskStartX; }
  void setCenterPosY(Pixel y) { data_.start_y = y & kMaskStartY; }
  void setRadiusX(Pixel radius_x)
  {
    data_.details_d = radius_x & kMaskDetailsD;
  }
  void setRadiusY(Pixel radius_y)
  {
    data_.details_e = radius_y & kMaskDetailsE;
  }
  void setCenterPos(Pixel x, Pixel y)
  {
    setCenterPosX(x);
    setCenterPosY(y);
  }
  void setRadius(Pixel radius_x, Pixel radius_y)
  {
    setRadiusX(radius_x);
    setRadiusY(radius_y);
  }

  Pixel getCenterPosX(void) const { return data_.start_x; }
  Pixel getCenterPosY(void) const { return data_.start_y; }
  Pixel getRadiusX(void) const { return data_.details_d; }
  Pixel getRadiusY(void) const { return data_.details_e; }
};
class Arc : public Graphic
{
 public:
  static constexpr Type kGraphicType = Type::kGraphicArc;  ///< 图形类型

  Arc(void) { setGraphicType(kGraphicType); }

  explicit Arc(
      const uint8_t name[3], Operation operation, Layer layer, Color color)
      : Graphic(name, operation, layer, color)
  {
    setGraphicType(kGraphicType);
  }

  explicit Arc(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel center_x, Pixel center_y, Pixel radius_x, Pixel radius_y,
      Ang start_ang, Ang end_ang)
      : Graphic(name, operation, layer, color)
  {
    setCenterPos(center_x, center_y);
    setRadius(radius_x, radius_y);
    setAng(start_ang, end_ang);
    setGraphicType(kGraphicType);
    setLineWidth(1);
  }

  explicit Arc(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel center_x, Pixel center_y, Pixel radius_x, Pixel radius_y,
      Ang start_ang, Ang end_ang, Pixel line_width)
      : Graphic(name, operation, layer, color)
  {
    setCenterPos(center_x, center_y);
    setRadius(radius_x, radius_y);
    setAng(start_ang, end_ang);
    setLineWidth(line_width);
    setGraphicType(kGraphicType);
  }

  void setCenterPosX(Pixel x) { data_.start_x = x & kMaskStartX; }
  void setCenterPosY(Pixel y) { data_.start_y = y & kMaskStartY; }
  void setRadiusX(Pixel radius_x)
  {
    data_.details_d = radius_x & kMaskDetailsD;
  }
  void setRadiusY(Pixel radius_y)
  {
    data_.details_e = radius_y & kMaskDetailsE;
  }
  void setStartAng(Ang start_ang)
  {
    data_.details_a = start_ang & kMaskDetailsA;
  }
  void setEndAng(Ang end_ang) { data_.details_b = end_ang & kMaskDetailsB; }
  void setCenterPos(Pixel x, Pixel y)
  {
    setCenterPosX(x);
    setCenterPosY(y);
  }
  void setRadius(Pixel radius_x, Pixel radius_y)
  {
    setRadiusX(radius_x);
    setRadiusY(radius_y);
  }
  void setAng(Ang start_ang, Ang end_ang)
  {
    setStartAng(start_ang);
    setEndAng(end_ang);
  }

  Pixel getCenterPosX(void) const { return data_.start_x; }
  Pixel getCenterPosY(void) const { return data_.start_y; }
  Pixel getRadiusX(void) const { return data_.details_d; }
  Pixel getRadiusY(void) const { return data_.details_e; }
  Ang getStartAng(void) const { return data_.details_a; }
  Ang getEndAng(void) const { return data_.details_b; }
};
class FloatingNumber : public Graphic
{
 public:
  static constexpr Type kGraphicType = Type::kGraphicFloating;  ///< 图形类型

  FloatingNumber(void) { setGraphicType(kGraphicType); }

  explicit FloatingNumber(
      const uint8_t name[3], Operation operation, Layer layer, Color color)
      : Graphic(name, operation, layer, color)
  {
    setGraphicType(kGraphicType);
  }

  explicit FloatingNumber(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel start_x, Pixel start_y, Pixel font_size, float value)
      : Graphic(name, operation, layer, color)
  {
    setStartPos(start_x, start_y);
    setFontSize(font_size);
    setDisplayValue(value);
    setGraphicType(kGraphicType);
    setLineWidth(font_size < 10 ? 1 : font_size / 10);
  }

  explicit FloatingNumber(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel start_x, Pixel start_y, Pixel font_size, float value,
      Pixel line_width)
      : Graphic(name, operation, layer, color)
  {
    setStartPos(start_x, start_y);
    setFontSize(font_size);
    setDisplayValue(value);
    setLineWidth(line_width);
    setGraphicType(kGraphicType);
  }

  void setStartPosX(Pixel x) { data_.start_x = x; }
  void setStartPosY(Pixel y) { data_.start_y = y; }
  void setFontSize(Pixel font_size) { data_.details_a = font_size; }
  void setDisplayValue(float value)
  {
    int32_t encode_value = (int32_t)(value * 1000);
    uint32_t *encode_value_ptr = (uint32_t *)(&encode_value);
    data_.details_c = (*encode_value_ptr) & kMaskDetailsC;
    data_.details_d = ((*encode_value_ptr) >> 10) & kMaskDetailsD;
    data_.details_e = ((*encode_value_ptr) >> 21) & kMaskDetailsE;
  }
  void setStartPos(Pixel x, Pixel y)
  {
    setStartPosX(x);
    setStartPosY(y);
  }

  Pixel getStartPosX(void) const { return data_.start_x; }
  Pixel getStartPosY(void) const { return data_.start_y; }
  Pixel getFontSize(void) const { return data_.details_a; }
  float getDisplayValue(void) const { return getEncodeValue() / 1000; }
  int32_t getEncodeValue(void) const
  {
    return (int32_t)(data_.details_c << 22 | data_.details_d << 11 |
                     data_.details_e);
  }
};
class Integer : public Graphic
{
 public:
  static constexpr Type kGraphicType = Type::kGraphicInteger;  ///< 图形类型

  Integer(void) { setGraphicType(kGraphicType); }

  explicit Integer(
      const uint8_t name[3], Operation operation, Layer layer, Color color)
      : Graphic(name, operation, layer, color)
  {
    setGraphicType(kGraphicType);
  }

  explicit Integer(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel start_x, Pixel start_y, Pixel font_size, int32_t value)
      : Graphic(name, operation, layer, color)
  {
    setStartPos(start_x, start_y);
    setFontSize(font_size);
    setDisplayValue(value);
    setGraphicType(kGraphicType);
    setLineWidth(font_size < 10 ? 1 : font_size / 10);
  }

  explicit Integer(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel start_x, Pixel start_y, Pixel font_size, int32_t value,
      Pixel line_width)
      : Graphic(name, operation, layer, color)
  {
    setStartPos(start_x, start_y);
    setFontSize(font_size);
    setDisplayValue(value);
    setLineWidth(line_width);
    setGraphicType(kGraphicType);
  }

  void setStartPosX(Pixel x) { data_.start_x = x; }
  void setStartPosY(Pixel y) { data_.start_y = y; }
  void setFontSize(Pixel font_size) { data_.details_a = font_size; }
  void setDisplayValue(int32_t value)
  {
    int32_t encode_value = (int32_t)(value);
    uint32_t *encode_value_ptr = (uint32_t *)(&encode_value);
    data_.details_c = (*encode_value_ptr) & kMaskDetailsC;
    data_.details_d = ((*encode_value_ptr) >> 10) & kMaskDetailsD;
    data_.details_e = ((*encode_value_ptr) >> 21) & kMaskDetailsE;
  }
  void setStartPos(Pixel x, Pixel y)
  {
    setStartPosX(x);
    setStartPosY(y);
  }

  Pixel getStartPosX(void) const { return data_.start_x; }
  Pixel getStartPosY(void) const { return data_.start_y; }
  Pixel getFontSize(void) const { return data_.details_a; }
  float getDisplayValue(void) const { return getEncodeValue(); }
  int32_t getEncodeValue(void) const
  {
    return (int32_t)(data_.details_c << 22 | data_.details_d << 11 |
                     data_.details_e);
  }
};
class String : public Graphic
{
 public:
  static constexpr Type kGraphicType = Type::kGraphicString;  ///< 图形类型

  String(void) { setGraphicType(kGraphicType); }

  explicit String(
      const uint8_t name[3], Operation operation, Layer layer, Color color)
      : Graphic(name, operation, layer, color)
  {
    setGraphicType(kGraphicType);
  }

  explicit String(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel start_x, Pixel start_y, Pixel font_size, size_t string_length)
      : Graphic(name, operation, layer, color)
  {
    setStartPos(start_x, start_y);
    setFontSize(font_size);
    setStringlength(string_length);
    setGraphicType(kGraphicType);
    setLineWidth(font_size < 10 ? 1 : font_size / 10);
  }

  explicit String(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel start_x, Pixel start_y, Pixel font_size, size_t string_length,
      Pixel line_width)
      : Graphic(name, operation, layer, color)
  {
    setStartPos(start_x, start_y);
    setFontSize(font_size);
    setStringlength(string_length);
    setLineWidth(line_width);
    setGraphicType(kGraphicType);
  }

  void setStartPosX(Pixel x) { data_.start_x = x & kMaskStartX; }
  void setStartPosY(Pixel y) { data_.start_y = y & kMaskStartY; }
  void setFontSize(Pixel font_size)
  {
    data_.details_a = font_size & kMaskDetailsA;
  }
  void setStringlength(size_t length)
  {
    data_.details_b =
        (length <= kMaxStringLength ? length : kMaxStringLength) &
        kMaskDetailsB;
  }
  void setStartPos(Pixel x, Pixel y)
  {
    setStartPosX(x);
    setStartPosY(y);
  }

  Pixel getStartPosX(void) const { return data_.start_x; }
  Pixel getStartPosY(void) const { return data_.start_y; }
  Pixel getFontSize(void) const { return data_.details_a; }
  size_t getLength(void) const
  {
    return data_.details_b <= kMaxStringLength
               ? data_.details_b
               : kMaxStringLength;
  }
};

class InterGraphicPackage : public InterAmongRobotsPackage
{
 public:
  virtual bool setSenderId(RfrId id) override
  {
    if (checkSenderId(id)) {
      sender_id_ = id;
      receiver_id_ = ids::RobotId2ClientId(id);
      return true;
    } else {
      return false;
    }
  }

  virtual bool setReceiverId(RfrId id) override
  {
    if (checkReceiverId(id)) {
      receiver_id_ = id;
      sender_id_ = ids::ClientId2RobotId(id);
      return true;
    } else {
      return false;
    }
  }

 protected:
  virtual bool checkSenderId(RfrId id) const override
  {
    return ids::IsRobotId(id);
  }

  virtual bool checkReceiverId(RfrId id) const override
  {
    return ids::IsClientId(id);
  }
};

class InterGraphicDeletePackage : public InterGraphicPackage
{
 public:
  typedef GraphicDeleteData Data;
  typedef DeleteOperation Operation;
  typedef GraphicLayer Layer;

  virtual CmdId getInterCmdId(void) const override { return 0x0100; }
  virtual DataLength getInterDataLength(void) const override
  {
    return sizeof(Data);
  }

  void setDeleteOperation(Operation operation)
  {
    data_.delete_operation = operation;
  }
  void setLayer(Layer layer) { data_.layer = layer; }

  Operation getDeleteOperation(void) const
  {
    return (Operation)data_.delete_operation;
  }
  Layer getLayer(void) const { return (Layer)data_.layer; }

 protected:
  Data data_ = {
      .delete_operation = Operation::kDeleteAll,
      .layer = Layer::kGraphicLayer0,
  };

  virtual void encodeInterData(uint8_t *data) override
  {
    memcpy(data, &data_, getInterDataLength());
  }
};

class InterGraphic1Package : public InterGraphicPackage
{
 public:
  typedef GraphicData Data;
  typedef GraphicType Type;
  typedef GraphicOperation Operation;
  typedef GraphicLayer Layer;
  typedef GraphicColor Color;

  virtual CmdId getInterCmdId(void) const override { return 0x0101; }
  virtual DataLength getInterDataLength(void) const override
  {
    return sizeof(Data);
  }

  void setStraightLine(const StraightLine &graphic_)
  {
    data_ = graphic_.getData();
  }
  void setRectangle(const Rectangle &graphic_) { data_ = graphic_.getData(); }
  void setCircle(const Circle &graphic_) { data_ = graphic_.getData(); }
  void setEllipse(const Ellipse &graphic_) { data_ = graphic_.getData(); }
  void setArc(const Arc &graphic_) { data_ = graphic_.getData(); }
  void setFloatingNumber(const FloatingNumber &graphic_)
  {
    data_ = graphic_.getData();
  }
  void setInteger(const Integer &graphic_) { data_ = graphic_.getData(); }

 protected:
  Data data_;

  virtual void encodeInterData(uint8_t *data) override
  {
    memcpy(data, &data_, sizeof(Data));
  }
};

class InterGraphic2Package : public InterGraphicPackage
{
 public:
  typedef GraphicData Data;
  typedef GraphicType Type;
  typedef GraphicOperation Operation;
  typedef GraphicLayer Layer;
  typedef GraphicColor Color;

  virtual CmdId getInterCmdId(void) const override { return 0x0102; }
  virtual DataLength getInterDataLength(void) const override
  {
    return sizeof(Data) * kMaxNumGraphicData_;
  }

  void setStraightLineAt(const StraightLine &graphic_, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic_.getData();
  }
  void setRectangleAt(const Rectangle &graphic_, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic_.getData();
  }
  void setCircleAt(const Circle &graphic_, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic_.getData();
  }
  void setEllipseAt(const Ellipse &graphic_, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic_.getData();
  }
  void setArcAt(const Arc &graphic_, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic_.getData();
  }
  void setFloatingNumberAt(const FloatingNumber &graphic_, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic_.getData();
  }
  void setIntegerAt(const Integer &graphic_, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic_.getData();
  }

 protected:
  static constexpr size_t kMaxNumGraphicData_ = 2;  ///< 最大图形数据数量

  Data data_[kMaxNumGraphicData_];

  virtual void encodeInterData(uint8_t *data) override
  {
    for (uint8_t i = 0; i < kMaxNumGraphicData_; i++) {
      memcpy(data + i * sizeof(Data), &data_[i], sizeof(Data));
    }
  }
};

class InterGraphic5Package : public InterGraphicPackage
{
 public:
  typedef GraphicData Data;
  typedef GraphicType Type;
  typedef GraphicOperation Operation;
  typedef GraphicLayer Layer;
  typedef GraphicColor Color;

  virtual CmdId getInterCmdId(void) const override { return 0x0103; }
  virtual DataLength getInterDataLength(void) const override
  {
    return sizeof(Data) * kMaxNumGraphicData_;
  }

  void setStraightLineAt(const StraightLine &graphic_, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic_.getData();
  }
  void setRectangleAt(const Rectangle &graphic_, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic_.getData();
  }
  void setCircleAt(const Circle &graphic_, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic_.getData();
  }
  void setEllipseAt(const Ellipse &graphic_, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic_.getData();
  }
  void setArcAt(const Arc &graphic_, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic_.getData();
  }
  void setFloatingNumberAt(const FloatingNumber &graphic_, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic_.getData();
  }
  void setIntegerAt(const Integer &graphic_, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic_.getData();
  }

 protected:
  static constexpr size_t kMaxNumGraphicData_ = 5;  ///< 最大图形数据数量

  Data data_[kMaxNumGraphicData_];

  virtual void encodeInterData(uint8_t *data) override
  {
    for (uint8_t i = 0; i < kMaxNumGraphicData_; i++) {
      memcpy(data + i * sizeof(Data), &data_[i], sizeof(Data));
    }
  }
};

class InterGraphic7Package : public InterGraphicPackage
{
 public:
  typedef GraphicData Data;
  typedef GraphicType Type;
  typedef GraphicOperation Operation;
  typedef GraphicLayer Layer;
  typedef GraphicColor Color;

  virtual CmdId getInterCmdId(void) const override { return 0x0104; }
  virtual DataLength getInterDataLength(void) const override
  {
    return sizeof(Data) * kMaxNumGraphicData_;
  }

  void setStraightLineAt(const StraightLine &graphic_, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic_.getData();
  }
  void setRectangleAt(const Rectangle &graphic_, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic_.getData();
  }
  void setCircleAt(const Circle &graphic_, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic_.getData();
  }
  void setEllipseAt(const Ellipse &graphic_, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic_.getData();
  }
  void setArcAt(const Arc &graphic_, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic_.getData();
  }
  void setFloatingNumberAt(const FloatingNumber &graphic_, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic_.getData();
  }
  void setIntegerAt(const Integer &graphic_, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic_.getData();
  }

 protected:
  static constexpr size_t kMaxNumGraphicData_ = 7;  ///< 最大图形数据数量

  Data data_[kMaxNumGraphicData_] = {0};

  virtual void encodeInterData(uint8_t *data) override
  {
    for (uint8_t i = 0; i < kMaxNumGraphicData_; i++) {
      memcpy(data + i * sizeof(Data), &data_[i], sizeof(Data));
    }
  }
};

class InterGraphicStringPackage : public InterGraphicPackage
{
 public:
  typedef GraphicData Data;
  typedef GraphicType Type;
  typedef GraphicOperation Operation;
  typedef GraphicLayer Layer;
  typedef GraphicColor Color;

  virtual CmdId getInterCmdId(void) const override { return 0x0110; }
  virtual DataLength getInterDataLength(void) const override
  {
    return sizeof(Data) + kMaxStringLength;
  }

  void setStrintg(const String &graphic, const std::string &characters)
  {
    data_ = graphic.getData();
    memset(characters_, 0, sizeof(characters_));
    memcpy(
        characters_, characters.c_str(),
        characters.size() > kMaxStringLength
            ? kMaxStringLength
            : characters.size());
  }

 protected:
  Data data_;
  uint8_t characters_[30];

  virtual void encodeInterData(uint8_t *data) override
  {
    memcpy(data, &data_, sizeof(Data));
    memset(data + sizeof(Data), 0, sizeof(characters_));
    memcpy(data + sizeof(Data), characters_, sizeof(characters_));
  }
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0301_INTER_GRAPHICS_HPP_ */
