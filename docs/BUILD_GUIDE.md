# 构建指南

本文说明如何从仓库构建两套固件：

```text
car-/                 STM32F103VET6 底盘/定位/按键/LED/OLED 固件
car_up_tools/         ESP32-S3 ESP-IDF 网页上位机
docs/                 协议、定位、按键/LED、构建说明
```

## 1. 仓库约定

- 主分支：`main`
- STM32 构建产物：`car-/MDK-ARM/stm/`，不提交
- ESP32 构建产物：`car_up_tools/build/`，不提交
- 应用源码、Keil 工程、ESP-IDF 工程、`sdkconfig`、文档、协议说明提交
- CMSIS/HAL 只提交本工程需要的部分，未使用的 DSP/NN/RTOS 组件已在 `.gitignore` 中排除

提交前检查：

```powershell
git status
git diff --check -- . ':!car-/Drivers' ':!car-/Core/Src/system_stm32f1xx.c' ':!car-/MDK-ARM/startup_stm32f103xe.s'
git add .
git commit -m "feat: ..."
```

> `car-/Drivers` 和 `system_stm32f1xx.c` 是 ST 原厂文件，保留了原始空白；上面的 `diff --check` 排除了这些 vendor 文件。`.gitattributes` 也已对 vendor 关闭 whitespace 检查。

发布固件时建议打 tag：

```powershell
git tag -a v0.1.0 -m "底盘 + ESP32 网页上位机"
git push origin main --tags
```

## 2. STM32 构建

### 2.1 环境

- Keil MDK 5.32 或更高
- ARM Compiler V5.06 update 7（工程使用 ARMCC 5）
- STM32F1xx Device Family Pack
- ST-Link 驱动/调试器

工程路径：

```text
car-/MDK-ARM/stm.uvprojx
```

### 2.2 编译

打开 Keil 工程，选择 `stm` target，`Build`。

命令行：

```powershell
& 'C:\Keil_v5\UV4\UV4.exe' -b 'D:\guangdian\car-\MDK-ARM\stm.uvprojx' -j0 -o build.log
Get-Content build.log -Tail 20
```

预期结果：

```text
Code=37512  RO-data=1092  RW-data=236  ZI-data=7708
0 Error(s), 0 Warning(s)
```

输出：

```text
car-/MDK-ARM/stm/stm.axf
car-/MDK-ARM/stm/stm.hex
```

### 2.3 烧录

Keil 中选择 ST-Link，`Download` 烧录 `stm.hex` 或 `stm.axf`。

注意：

- 板卡实际芯片是 STM32F103VET6（512 KB Flash），Keil 工程当前仍按 STM32F103VD/384 KB 设备配置；代码约 37 KB，不影响构建和烧录。
- 参数页固定使用 `0x0805F800`（384 KB 空间的最后一页），在 VET6 的 512 KB 空间内同样有效。
- `car-/stm.ioc` 仍是旧的 I2C1 PB8/PB9、100 kHz；如果重新用 CubeMX 生成，必须手动改回：
  - I2C1：PB6/PB7，400 kHz
  - I2C2：PB10/PB11，400 kHz
  - PB12/PB13：GPIO 输出，作为两片 ToF XSHUT
  - USART1 NVIC 使能
  - 新增源文件重新加入 Keil 分组

### 2.4 外设地址

| 设备 | 地址 |
|---|---|
| MPU6050（AD0 高） | 0x69（驱动兼容 0x68） |
| SSD1306 OLED | 0x3C |
| 前向 TOF400C | 0x60（8 位地址） |
| 右向 TOF400C | 0x62（8 位地址） |

I2C1/I2C2 都需要 2.2~4.7 kΩ 上拉。

## 3. ESP32-S3 构建

### 3.1 环境

- ESP-IDF v5.4.4
- ESP32-S3 目标
- 2 MB Flash（当前 `sdkconfig` 为 2 MB、single app）

本机 EIM 安装的激活脚本：

```powershell
& 'C:\Espressif\tools\Microsoft.v5.4.4.PowerShell_profile.ps1'
```

如果你的 ESP-IDF 安装路径不同，使用自己的 `export.ps1`/EIM profile，确保：

```text
IDF_PATH
IDF_PYTHON_ENV_PATH
idf.py 可用
```

### 3.2 编译

仓库提交了 `sdkconfig.defaults`，`sdkconfig` 本身按 ESP-IDF 惯例不提交。

首次克隆后如果 `car_up_tools/sdkconfig` 不存在：

```powershell
cd D:\guangdian\car_up_tools
idf.py set-target esp32s3
idf.py build
```

已有本地 `sdkconfig` 时直接：

```powershell
cd D:\guangdian\car_up_tools
idf.py build
```

预期：

```text
car_up_tools.bin binary size 约 0xD38D0 字节
Project build complete
```

输出：

```text
car_up_tools/build/bootloader/bootloader.bin
car_up_tools/build/partition_table/partition-table.bin
car_up_tools/build/car_up_tools.bin
```

### 3.3 烧录与监视

```powershell
idf.py -p COMx flash monitor
```

如果端口不确定：

```powershell
idf.py -p COMx monitor
```

退出 monitor：`Ctrl+]`。

### 3.4 接线

| ESP32-S3 | STM32 |
|---|---|
| GPIO17 TX | PA10 USART1 RX |
| GPIO18 RX | PA9 USART1 TX |
| GND | GND |

> STM32 的 USART1 原来用于 USB-TTL 调试。接 ESP32 后不要再同时接 USB-TTL 的 TX，避免冲突。

### 3.5 网页

```text
SSID:     Crawler-Car
Password: 12345678
URL:      http://192.168.4.1/
```

浏览器连上 AP 后打开该地址。

## 4. 构建产物与 Git

`.gitignore` 已排除：

- `car-/MDK-ARM/stm/`
- `car_up_tools/build/`
- `.uvguix.*`、`.uvoptx`
- `__pycache__`
- 未使用的 CMSIS DSP/NN/RTOS 组件

发布时只需要提交源码和文档；固件二进制通过 Release 附件或构建产物发布，不建议直接提交到主分支。

## 5. 版本对应的协议

- STM32 与 ESP32 必须使用同一版 `CONTROL_PROTOCOL.md` 的帧定义
- 当前协议版本：`CHASSIS_PROTOCOL_VERSION = 1`
- 修改帧长度、字段顺序、缩放系数时：
  1. 先改 `docs/CONTROL_PROTOCOL.md`
  2. 再改 STM32 `chassis_protocol.c`
  3. 再改 ESP32 `web_server.c` / `car_state.c`
  4. 最后同步网页 `app.js`
  5. 打 tag 并更新构建指南

## 6. 上车验证顺序

1. 只烧 STM32，USB-TTL 发 `0x04 [1,1]` 使能，确认电机方向。
2. 发 `0x01` 低速，确认四轮转速和编码器符号。
3. 接 OLED，确认正常页显示车速/角度/位置。
4. 按 K4 切调试页，按 K1/K2 验证使能和模式。
5. 按 K3 做 MPU6050 零偏标定，保持静止，确认 LED 结果。
6. 接两片 ToF，扫描 0x60/0x62，做多点标定。
7. 接 ESP32，连 AP，确认网页 `/api/state` 有数据。
8. 网页摇杆、点走直线、矩形、圆、手绘。
9. 比赛模式打开 ToF，测试遮挡/恢复和路径偏差中止。

## 7. 常见问题

| 现象 | 排查 |
|---|---|
| Keil 链接报错 | 确认新增 `.c` 文件在 `Application/User/Nav` 分组 |
| MPU6050 读不到 | 确认 AD0 高电平，地址 0x69；I2C1 PB6/PB7 上拉 |
| OLED 不亮 | 地址 0x3C，I2C1 400 kHz，上拉电阻 |
| 两片 ToF 冲突 | 确认 PB12/PB13 XSHUT，先初始化前向再右向 |
| ESP32 编译找不到 IDF | 先运行 EIM profile 或 `export.ps1` |
| ESP32 能连 AP 但没数据 | 检查 GPIO17/18 交叉、共地、115200 8N1 |
| 保存参数失败 | 先断使能，再保存；Flash 页写保护/地址检查 |
| 网页圆走成折线 | 当前实现按小直线段近似，ARC 图元保留给后续绝对圆心圆弧 |
