# OLED / LCD 使用文档

> 适用板卡：**DJI_Board_TypeA**（STM32F427）  
> OLED 型号：128×64 像素，I²C 接口（**注意区分 SSD1306 / SH1106，见第 8 节**）  
> 库路径：`shared/libraries/oled.h / oled.cc`

---

## 1. 硬件接线

| OLED 引脚 | 板卡引脚 | 说明 |
|-----------|---------|------|
| VCC       | 3.3 V   |      |
| GND       | GND     |      |
| SCL       | I2C2_SCL | TypeA 使用 `hi2c2` |
| SDA       | I2C2_SDA |      |

I²C 地址：`0x3C`（默认，部分模块为 `0x3D`，看 SA0 引脚）

---

## 2. 初始化

```cpp
#include "i2c.h"
#include "oled.h"

#define OLED_COL_OFFSET 2  // SH1106 用 2，真 SSD1306 用 0，见第 8 节

static display::OLED* OLED = nullptr;

void RM_RTOS_Init(void) {
  OLED = new display::OLED(&hi2c2, 0x3C, OLED_COL_OFFSET);
}
```

> **注意**：必须 `#include "i2c.h"` 才能让 `hi2c2` 可见，否则 C++ 编译器会报未定义错误。

---

## 3. 常用 API

### 3.1 清屏 / 填充

```cpp
OLED->OperateGram(display::PEN_CLEAR);     // 全屏清零（黑）
OLED->OperateGram(display::PEN_WRITE);     // 全屏置1（白）
OLED->OperateGram(display::PEN_INVERSION); // 全屏像素取反
```

### 3.2 刷新到屏幕

所有绘制操作只写到内存 gram，**必须调用 `RefreshGram()` 才会真正输出到屏幕**。

```cpp
OLED->RefreshGram();
```

### 3.3 写文字

```cpp
OLED->Printf(row, col, fmt, ...);
```

| 参数 | 说明 |
|------|------|
| `row` | 文本行，0~4（共 5 行，每行高 12 px，小字体） |
| `col` | 文本列（字符单位），建议从 `2` 开始以避免左侧像素被遮挡 |
| `fmt` | printf 格式字符串 |

示例：

```cpp
OLED->Printf(0, 2, "Hello World");
OLED->Printf(1, 2, "Val: %d", some_int);
OLED->RefreshGram();
```

> **左边距建议**：屏幕物理左侧约有 1 个字符宽（6 px）可能被外壳遮挡，  
> 建议 `col` 从 `2` 开始（留 12 px 边距）确保文字完整可见。

### 3.4 画点 / 画线

```cpp
OLED->DrawPoint(x, y, pen);                     // 单点，x∈[0,127] y∈[0,63]
OLED->DrawLine(x1, y1, x2, y2, pen);            // 直线
```

`pen` 取值：

| 值 | 效果 |
|----|------|
| `display::PEN_CLEAR` | 置 0（黑） |
| `display::PEN_WRITE` | 置 1（白） |
| `display::PEN_INVERSION` | 像素取反（高亮） |

### 3.5 内置动画

```cpp
OLED->ShowRMLOGO();        // 显示 RoboMaster logo
OLED->ShowIlliniRMLOGO();  // 显示 Illini RM logo
OLED->DrawCat();           // 彩虹猫动画（需在循环中每帧调用）
```

---

## 4. 高亮反选（菜单选中效果）

通过对选中行所有像素逐行取反，可实现白底黑字的高亮效果：

```cpp
// 对第 text_row 行（0-based，每行12px）做像素取反
static void InvertTextRow(uint8_t text_row) {
  const uint8_t y_start = text_row * 12;
  const uint8_t y_end   = y_start + 11;
  for (uint8_t y = y_start; y <= y_end; ++y) {
    OLED->DrawLine(0, y, 127, y, display::PEN_INVERSION);
  }
}
```

调用顺序：先 `Printf` 写文字（白字黑底），再 `InvertTextRow` 取反，最后 `RefreshGram`。

---

## 5. 按键输入（**注意：分两种读法**）

模块上四个按键接在 **PC0~PC3**，每个键都是「一端接引脚、另一端接 GND，外部上拉」的独立按键，
松开读高（ADC ~4095）、按下读低（ADC ~0）。

**关键坑：这四个脚被 CubeMX 分成了两种模式，不能用同一套 API 读。**

| 键 | 引脚 | `gpio.c` / `adc.c` 里的模式 | 读法 |
|----|------|------------------------|------|
| 上 | PC0 | `GPIO_MODE_ANALOG`（ADC1_IN10，rank 2） | 读 ADC，阈值判断 |
| 下 | PC1 | `GPIO_MODE_ANALOG`（ADC1_IN11，rank 3） | 读 ADC，阈值判断 |
| #  | PC2 | `GPIO_MODE_INPUT` + `GPIO_PULLUP`（`L1_Pin`） | `HAL_GPIO_ReadPin`，低有效 |
| *  | PC3 | `GPIO_MODE_INPUT` + `GPIO_PULLUP`（`M1_Pin`） | `HAL_GPIO_ReadPin`，低有效 |

> **`GPIO_MODE_ANALOG` 会关断数字输入缓冲器**，所以 PC0 / PC1 的 `IDR` 恒为 0，
> 用 `HAL_GPIO_ReadPin` 读它们永远得不到按键状态。这两个键只能走 ADC。
> 想改回数字模式也可以，但要先停掉 ADC；由于本工程没有其他地方使用 `hadc1`，
> 两种做法都可行，走 ADC 改动最小。

### 5.1 ADC 初始化（PC0 / PC1）

`MX_DMA_Init()` 和 `MX_ADC1_Init()` 在 RTOS 启动前已由板级代码调用，只需开启 DMA 搬运：

```cpp
#include "adc.h"

#define ADC_RANK_COUNT 4
// HAL_ADC_MspInit 里 DMA 配的是 HALFWORD 搬运，缓冲区必须是 16 位！
// 用 uint32_t 的话 4 次转换会挤进前两个元素，rank 3/4 永远取不到值。
static uint16_t adc_buf[ADC_RANK_COUNT];

HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_RANK_COUNT);
```

扫描序列顺序（见 `adc.c`）：

| DMA 下标 | rank | 通道 | 引脚 | 用途 |
|---------|------|------|------|------|
| 0 | 1 | `ADC_CHANNEL_14` | PC4 | 参考分压，常态 ~2174 |
| 1 | 2 | `ADC_CHANNEL_10` | PC0 | **上键** |
| 2 | 3 | `ADC_CHANNEL_11` | PC1 | **下键** |
| 3 | 4 | `ADC_CHANNEL_6`  | PA6 | `OLED_BUTTON`（本模块未使用） |

### 5.2 四个键的读取

```cpp
#define ADC_PRESS_LEVEL 2048  // 静止 ~4095，按下 ~0，取中点即可

static bool KeyUp(void)   { return adc_buf[1] < ADC_PRESS_LEVEL; }
static bool KeyDown(void) { return adc_buf[2] < ADC_PRESS_LEVEL; }
static bool KeyHash(void) { return HAL_GPIO_ReadPin(L1_GPIO_Port, L1_Pin) == GPIO_PIN_RESET; }
static bool KeyStar(void) { return HAL_GPIO_ReadPin(M1_GPIO_Port, M1_Pin) == GPIO_PIN_RESET; }
```

PC2 / PC3 在 `MX_GPIO_Init()` 里已经配好 `INPUT` + `PULLUP`，**不要再调 `HAL_GPIO_Init` 去重配 GPIOC**，
那样会把 PC0 / PC1 从模拟模式踢出去、破坏 ADC 采样。

### 5.3 消抖与边沿检测

```cpp
typedef struct { bool prev; uint32_t last_ms; } KeyState;

static bool JustPressed(bool now_down, KeyState* s, uint32_t now_ms) {
  bool fired = false;
  if (now_down && !s->prev && (now_ms - s->last_ms >= 200)) {
    s->last_ms = now_ms;
    fired = true;
  }
  s->prev = now_down;
  return fired;
}
```

> `GPIO_InitTypeDef` 不要用 `= {0}` 零初始化，工程开了
> `-Werror=missing-field-initializers`，必须把每个字段显式赋值。

### 5.4 找引脚的辅助程序

`vehicles/Dart/LCD_Input_Probe.cc`（target `lcd_probe`）是一个只读 `IDR` 的引脚探测程序：
开机自学 3 秒排除自发翻转的引脚，之后按下任意按键，屏幕会列出发生跳变的引脚名。
换了模块或改了接线时可以用它重新定位按键，全程不驱动任何输出，对板上
`MOS_CTL1~4` / `LASER` / `Q1` / `Q2` 等外设无风险。

## 6. 完整菜单示例框架

`vehicles/Dart/LCD_Test.cc` 中实现了一个两条目的交互菜单，逻辑如下：

```
启动
  └─ 显示 RM LOGO（2s）
  └─ 显示 Illini RM LOGO（2s）
  └─ 进入 STATE_MENU
        上键 → cursor 上移，高亮选中行
        下键 → cursor 下移，高亮选中行
        #  → 进入 STATE_DETAIL，显示对应详情页
  └─ STATE_DETAIL
        *  → 返回 STATE_MENU
```

渲染层（DrawMenu / DrawDetail）每次都先 `OperateGram(PEN_CLEAR)` 清屏再重绘，  
通过 `need_redraw` 标志避免无谓刷新。

---

## 7. 注意事项

1. **`hi2c2` vs `hi2c1`**：TypeA 板 OLED 接在 I2C2，TypeC 等其他板请查对应 `.ioc` 文件。
2. **每次改动绘制内容后都需要 `RefreshGram()`**，否则屏幕不更新。
3. **`DrawCat()` 会自带清屏**，如需在动画帧上叠加文字，需在每帧 `DrawCat()` 后再次调用 `Printf` + `RefreshGram`。
4. **屏幕坐标原点**在左上角，X 轴向右（0~127），Y 轴向下（0~63）。

---

## 8. SH1106 与 SSD1306 的列偏移（**踩过的坑**）

### 8.1 症状

Dart 上这块标称「1306」的模块，实际控制器是 **SH1106**。按 SSD1306 驱动会同时出现两个现象：

- 屏幕**最右侧有一条约 2 像素宽的竖条**，内容随机、不随绘图改变
- 所有内容**整体左移 2 像素**，最左边一列文字被切掉（例如 `*:Back` 的 `*` 缺一半）

### 8.2 原因

| | SSD1306 | SH1106 |
|---|---|---|
| GDDRAM 列数 | 128 | **132** |
| 面板接线 | SEG0~SEG127 | 通常 **SEG2~SEG129** |

`SetPos` 若从第 0 列开始写 128 字节：

- 我们的第 0、1 列写进了 SEG0、SEG1 —— **面板没接，看不见**
- 控制器的第 128、129 列**从未被写入**，保持上电随机值 —— 就是那条竖条

### 8.3 判定方法

开机时整屏 `PEN_WRITE` 刷白、再整屏 `PEN_CLEAR` 刷黑：

- 竖条**不跟随**（全黑时仍是白线、全白时有零星黑点）→ 它在可寻址范围之外，**是 SH1106**
- 竖条**跟随**屏幕明暗 → 在 gram 内，是绘图代码自己画出来的，与本节无关

### 8.4 解法

`display::OLED` 构造函数的第三个参数是列偏移，**默认 0，不影响既有代码**：

```cpp
// SH1106
OLED = new display::OLED(&hi2c2, 0x3C, 2);
// 真 SSD1306（可省略第三个参数）
OLED = new display::OLED(&hi2c2, 0x3C);
```

内部实现只是在 `SetPos` 里对列地址加上偏移：

```cpp
void OLED::SetPos(uint8_t x, uint8_t y) {
  x += col_offset_;
  ...
}
```

> **不要把偏移硬写进库里。** `shared/libraries/oled.cc` 是全仓库共享的，
> 其他车上如果是真 SSD1306，加了偏移反而会让画面整体右移 2 像素。
