# 工程总览：按键、LED、OLED、调试窗口与控制帧

本文是这次新增功能的工程说明，配合：

- [CONTROL_PROTOCOL.md](CONTROL_PROTOCOL.md)：完整串口帧
- [NAVIGATION.md](NAVIGATION.md)：定位/EKF/ToF
- [../car_up_tools/README.md](../car_up_tools/README.md)：ESP32 网页上位机

## 1. 系统组成

```text
手机/电脑浏览器
      │ Wi-Fi AP
      ▼
ESP32-S3：网页 + HTTP API + 调试窗口
      │ UART1 115200 8N1
      ▼
STM32F103VET6：底盘 PID + 定位 + 路径 + OLED + 按键/LED
      │
      ├─ TB6612 × 4
      ├─ 编码器 × 4
      ├─ MPU6050（AD0 高，0x69）
      ├─ TOF400C / VL53L1X × 2（I2C2）
      └─ SSD1306 OLED（I2C1，0x3C）
```

## 2. 板载按键与 LED

### 引脚

| 功能 | 引脚 | 说明 |
|---|---|---|
| LED0 | PC0（15 脚） | 共阳极，低电平点亮 |
| LED1 | PC1（16 脚） | 共阳极，低电平点亮 |
| LED2 | PC2（17 脚） | 共阳极，低电平点亮 |
| LED3 | PC3（18 脚） | 共阳极，低电平点亮 |
| K1 | PC4（33 脚） | 输入，内部上拉，按下为低 |
| K2 | PC5（34 脚） | 输入，内部上拉，按下为低 |
| K3 | PC6（63 脚） | 输入，内部上拉，按下为低 |
| K4 | PC7（64 脚） | 输入，内部上拉，按下为低 |

### 按键功能

| 按键 | 短按功能 | 说明 |
|---|---|---|
| K1 / PC4 | 使能 / 断使能 | 断使能时先 `PATH_ABORT` 并零速；使能时清故障 |
| K2 / PC5 | 日常模式 / 比赛模式 | 日常模式 ToF 只显示/标定，不参与 EKF |
| K3 / PC6 | MPU6050 零偏标定 | 立即停车、断使能，OLED 显示 CAL，采样 200 次（约 1 s） |
| K4 / PC7 | OLED 正常页 / 调试页 | 正常页显示车速、角度、位置；调试页显示详细状态 |

K3 标定完成后**不会自动使能**，需要再按 K1，避免标定时误动。

### LED 状态

| LED | 含义 |
|---|---|
| PC0 | 500 ms 心跳闪烁 |
| PC1 | 常亮 = 底盘已使能 |
| PC2 | 常亮 = 存在故障；标定成功灭，失败闪 2 s |
| PC3 | 常亮 = OLED 处于调试页 |

> 共阳极：`GPIO_PIN_RESET` 点亮，`GPIO_PIN_SET` 熄灭。上电初始化先写高，避免亮一下。

## 3. OLED 显示

### 正常页（非 Debug）

```text
SPD  25 CM/S
TH   12 DEG
X   123 CM
Y   234 CM
MODE DAILY
TOF 3 IMU OK
K1 EN  K2 MODE
K3 CAL K4 DEBUG
```

- 显示车速（cm/s）、航向角（deg）、场地 x/y（cm）
- 状态变化：模式/页面/标定状态切换时整屏清一次
- 每个页面在重绘前单独清空，避免数字变短后残留旧像素

### 调试页

```text
X 123 CM Y 234 CM
TH  12  V  25
TOF F 1234 R 2345
IMU OK TOF 3
PXX 25 YY 25
M1 100 M2 100
M3 100 M4 100
EKF OK UPD 12
```

### 标定页

```text
MPU6050
CALIBRATING
KEEP STILL
CAR DISABLED
```

## 4. ESP32 网页调试窗口

网页底部新增 **Debug / 错误窗口**：

- 连接/断开 ESP32
- HTTP 请求失败
- 命令被拒绝
- 底盘故障码变化
- 运动状态变化（IDLE / RUN / PAUSE / ERROR）
- 参数操作失败
- ToF 标定失败
- JS 运行时错误
- “清空”按钮

窗口最多保留 120 行，自动滚动到底部。

## 5. 控制帧摘要

完整字节布局见 [CONTROL_PROTOCOL.md](CONTROL_PROTOCOL.md)。

```text
AA 55 | LEN | SEQ | TYPE | PAYLOAD | CRC16_L | CRC16_H
```

| TYPE | 功能 |
|---|---|
| `0x01` | 速度控制（摇杆） |
| `0x02` | 四轮转速 |
| `0x03` | 四轮 PID |
| `0x04` | 使能/清故障 |
| `0x05` | 心跳 |
| `0x06` | 停止/断使能 |
| `0x10` | 日常/比赛模式 |
| `0x11` | LINE_TO 绝对点 |
| `0x12` | TURN_TO |
| `0x13` | ARC |
| `0x14` | NAV_STOP |
| `0x15` | 速度上限 |
| `0x16` | PATH_BEGIN |
| `0x17` | PATH_COMMIT |
| `0x18` | PATH_ABORT |
| `0x19` | ToF 多点标定 |
| `0x1A` | 参数读写/保存/加载 |
| `0x81` | 底盘状态 |
| `0x82` | 定位状态 |
| `0x83` | 路径状态 |
| `0x84` | 参数回读 |
| `0x85` | 标定回读 |

## 6. 已完成效果

- 四电机速度 PID、编码器测速、TB6612 安全换向
- MPU6050 + 轮速 + 双 ToF 的 6 状态 EKF
- 4 m×4 m 墙线模型、ToF 新息门控、遮挡/障碍物拒绝
- 日常模式不融合 ToF；比赛模式融合
- 网页显示场地、箭头、轨迹、规划路径
- 摇杆、点走直线、直线/矩形/圆/手绘折线
- 原地转向
- 轨迹偏差监视，连续超差自动 `PATH_ABORT`
- 速度/PID/模式/ToF 参数在线设置
- ToF 多点线性标定，STM32 Flash 持久化
- ESP32 NVS 备份 7 个运行参数
- 4 按键 + 4 LED + OLED 正常/调试/标定页面
- ESP32 网页 Debug/错误窗口

## 7. 重要说明

### MPU6050 标定不是“磁偏”标定

MPU6050 只有加速度计和陀螺仪，没有磁力计。K3 标定的是：

- 陀螺仪静止零偏（gyro bias）
- 标定期间强制停车、断使能

它不能修正磁偏角/绝对航向。如果需要绝对航向，需要额外磁力计（如 QMC5883L/HMC5883L），MPU6050 本身做不到。

### 参数保存

- 保存到 STM32 Flash 要求底盘已断使能
- 网页“备份到 ESP32”保存最近回读的 7 个参数（速度/模式/ToF）
- 四轮 PID 保存在 STM32 Flash；“从 ESP32 恢复”不会恢复 PID，除非后续增加 PID 备份

### 圆形/手绘

ESP32 目前把圆和手绘拆成小直线段下发，STM32 的 `0x13 ARC` 图元已经保留，供后续做绝对圆心圆弧。

## 8. 编译与烧录

STM32：

```text
Keil: MDK-ARM/stm.uvprojx
Code=37512  RO=1092  RW=236  ZI=7708
0 Error(s), 0 Warning(s)
```

ESP32-S3：

```powershell
& 'C:\Espressif\tools\Microsoft.v5.4.4.PowerShell_profile.ps1'
cd D:\guangdian\car_up_tools
idf.py build
idf.py -p COMx flash monitor
```

AP：

```text
SSID: Crawler-Car
Password: 12345678
http://192.168.4.1/
```
