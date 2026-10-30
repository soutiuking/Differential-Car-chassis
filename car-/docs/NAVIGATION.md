# 4 m × 4 m 场地定位与显示说明

本文件对应新增的导航层代码：

- MPU6050 100 Hz 姿态/航向输入
- 四路轮速 100 Hz 航位推算
- 两片 TOF400C（VL53L1X）前向 + 右向墙距
- 6 状态 EKF 融合
- 0.96 寸 SSD1306 OLED 状态显示
- USART1 新增 `0x82` 定位帧

这一版只做**位姿估计 + ToF 修正 + 显示 + 上位机回传**，不直接接管底盘运动控制。

---

## 1. 硬件连接

### I2C1：MPU6050 + OLED

| 信号 | 引脚 | 说明 |
|---|---|---|
| I2C1_SCL | PB6 | 400 kHz |
| I2C1_SDA | PB7 | 400 kHz |
| MPU6050 | 地址 0x69（AD0 高，实物配置） | 3.3 V；驱动同时兼容 AD0 低的 0x68 |
| SSD1306 OLED | 地址 0x3C | 128×64，3.3 V |

I2C1 已从原来的 PB8/PB9 重映射改回接线表上的 **PB6/PB7**，速率提高到 **400 kHz**。总线上必须有 2.2~4.7 kΩ 上拉电阻。

### I2C2：两片 TOF400C

| 信号 | 引脚 | 说明 |
|---|---|---|
| I2C2_SCL | PB10 | 400 kHz |
| I2C2_SDA | PB11 | 400 kHz |
| 前向 XSHUT | PB12 | 低复位，高使能 |
| 右向 XSHUT | PB13 | 低复位，高使能 |

两片 VL53L1X 上电默认地址都是 `0x29`（8 位地址 `0x52`）。初始化时：

1. 两片 XSHUT 同时拉低；
2. 前向释放，初始化后改地址为 `0x60`；
3. 右向释放，初始化后改地址为 `0x62`；
4. 前向先启动测距，125 ms 后右向启动，避免两束激光同时发射。

### 安装坐标

车体坐标：`+x` 向前，`+y` 向左，逆时针为航向正方向。

| ToF | 安装位置（车体，m） | 射线方向 |
|---|---|---|
| 前向 | `(0.080, 0.000)` | `0 rad` |
| 右向 | `(0.000, -0.060)` | `-π/2` |

场地坐标原点在**左下角**，`x` 向右、`y` 向上，范围 `0~4 m`。上电车头朝 `+x`，起始位置默认 `(0.5, 0.5)`；如果两片 ToF 都能在启动时读到有效墙距，会用墙距反算初始 `x/y`。

这些值都在 `Core/Inc/nav_config.h` 中，装车后按实测修改。

---

## 2. 数据流

```text
MPU6050 ──100 Hz──┐
四路轮速 ─100 Hz──┼─> EKF 预测 ─> 6 状态位姿 ─> nav_vars ─> OLED / USART1 0x82
前向 ToF ─~4 Hz───┤        │
右向 ToF ─~4 Hz───┘        └─> 墙线预测 + 新息门控 + 逐轴更新
```

### EKF 状态

```text
[x, y, theta, v, omega, gyro_bias]
```

- `x, y`：场地坐标，m
- `theta`：航向，rad
- `v`：车体线速度，m/s
- `omega`：车体角速度，rad/s
- `gyro_bias`：陀螺残余零偏，rad/s

### 预测

- 轮速左右平均得到 `v`
- 左右轮速差得到 `omega`
- 陀螺 yaw rate 参与 `omega/bias` 更新
- 100 Hz 预测，ToF 每片约 4 Hz 更新（100 ms 测距 + 250 ms 周期间隔，两片错开）

### ToF 更新

每片 ToF 的测量模型是“传感器射线与 4 面墙求交”：

```text
传感器世界位置 = 车体位姿 + R(theta) * 安装偏移
射线方向       = theta + 安装角
预测距离       = 射线与墙线段的交点距离
```

门控：

- 实测距离与预测距离残差超过 `0.10 m + 4σ`：拒绝；
- 实测明显小于预测（再超过 0.02 m）：判为障碍物，不更新位置；
- 靠近墙角、射线打到墙段端点：判为无模型，不更新；
- x 有效只更新 x，y 有效只更新 y，不要求两片同时有效；
- 800 ms 没有有效更新，该轴标记失效。

### 失效降级

- 单轴失效：另一轴继续更新；
- 双轴失效：继续轮速/陀螺航位推算，协方差增大；
- 长时间失效：显示 `TOF` 无效，但不会自动停车（这一版不做导航闭环）；
- 障碍物：只做标记，位置不更新。

---

## 3. 任务周期

| 任务 | 周期 | 位置 |
|---|---:|---|
| 电机 100 Hz 控制 | 10 ms | **SysTick 中断** |
| MPU6050 读取/EKF 预测 | 10 ms | 主循环 |
| ToF 轮询 | 10 ms 交替；有效测量约 4 Hz/片 | 主循环 |
| OLED 分页刷新 | 20 ms/页 | 主循环 |
| USART1 状态帧 0x81 | 100 ms | 主循环 |
| USART1 定位帧 0x82 | 100 ms | 主循环 |

电机控制已经移到 SysTick 中断里分频到 100 Hz，因此 OLED 分页写 I2C（约 3 ms）不会拖慢电机环。

---

## 4. 新增串口定位帧 0x82

帧格式与底盘协议一致：

```text
AA 55 | LEN | SEQ | TYPE | PAYLOAD | CRC16_L | CRC16_H
```

`TYPE=0x82`，`LEN=30`，小端。载荷布局：

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | u16 | flags |
| 2 | i16 | x，mm |
| 4 | i16 | y，mm |
| 6 | i16 | theta，mrad |
| 8 | i16 | v，mm/s |
| 10 | i16 | omega，mrad/s |
| 12 | i16 | roll，mrad |
| 14 | i16 | pitch，mrad |
| 16 | u16 | 前向 ToF，mm |
| 18 | u16 | 右向 ToF，mm |
| 20 | u16 | Pxx，cm² |
| 22 | u16 | Pyy，cm² |
| 24 | u16 | Ptt，mrad² |
| 26 | u16 | 前向 ToF 有效更新次数低 16 位 |
| 28 | u16 | 右向 ToF 有效更新次数低 16 位 |

`flags`：

| 位 | 含义 |
|---|---|
| 0 | EKF 正常 |
| 1 | IMU 最近读取正常 |
| 2 | 前向 ToF 有效 |
| 3 | 右向 ToF 有效 |
| 4 | 前向 ToF 判为障碍物 |
| 5 | 右向 ToF 判为障碍物 |

示例（x=0.5 m、y=0.5 m、两路 ToF 均有效）：

```text
AA 55 1E 02 82 0F 00 F4 01 F4 01 00 00 00 00 00 00 00 00 00 00 5C 0D B8 01 19 00 19 00 64 00 00 00 00 00 F3 56
```

`tools/chassis_host.py monitor` 会同时解析 `0x81` 和 `0x82`。

---

## 5. 参数与标定

### 必须标定

1. **轮径/轮距**：`chassis_config.h` 中的 `CHASSIS_WHEEL_DIAMETER_M`、`CHASSIS_TRACK_WIDTH_M`。
2. **编码器方向**：四个 `CHASSIS_ENCODER_SIGN_Mx` 必须使前进时为正。
3. **电机方向**：四个 `CHASSIS_MOTOR_SIGN_Mx` 必须使正命令前进。
4. **ToF 安装偏移/角度**：`nav_config.h` 中 `NAV_TOF_*_OFFSET_*`、`NAV_TOF_*_ANGLE_RAD`。
5. **场地尺寸**：`NAV_FIELD_LENGTH_M`、`NAV_FIELD_WIDTH_M`，测试场地实测后修改。
6. **起始位姿**：默认 `(0.5, 0.5)`、车头 `+x`；如果两路 ToF 启动时都有效，会自动反算 x/y。

### ToF 参数

```c
#define TOF_TIMING_BUDGET_MS    100U
#define TOF_INTER_MEAS_MS       250U
#define TOF_PHASE_OFFSET_MS     125U
```

- 100 ms 是响应速度和 4 m 量程的折中；
- 如果远处墙经常读不到，把 timing budget 调到 200 ms，同时把 inter-measurement 调到 450 ms，并保持两片错开半个周期；
- 两片周期相同但启动错开 125 ms，正常情况下不会同时发射；
- 如果现场有其他 ToF 小车，仍可能偶发串扰，靠新息门控拒绝。

### EKF 噪声

`nav_config.h` 中：

```c
NAV_EKF_Q_POS / Q_THETA / Q_V / Q_OMEGA / Q_BIAS
NAV_EKF_R_WHEEL_V / R_WHEEL_OMEGA / R_GYRO
NAV_EKF_R_TOF_BASE / R_TOF_SCALE
```

建议顺序：

1. 先让小车静止，确认陀螺零偏和 `omega` 不漂；
2. 手推直线，比较轮速里程和卷尺距离；
3. 手推转弯，比较陀螺航向和实际角度；
4. 打开 ToF，观察 `Pxx/Pyy` 是否收敛；
5. 最后调 `Q` 和 `R`，不要一开始就大幅修改。

---

## 6. 显示内容

OLED 8 页，每页 20 ms，整屏约 160 ms 更新一次：

```text
X 050CM Y 050CM
TH   0 V   0
TOF F3420 R 440
IMUOK TOF3
PXX 25 YY 25
M1  100 M2  100
M3  100 M4  100
EKFOK UPD 12
```

- `TOF` 行显示原始毫米值；末尾有效性看第 3 页 `TOF` 后的 0~3 掩码；
- 第 3 页 `IMUOK` 表示最近一次 MPU 读取成功；
- `PXX/PYY` 是位置协方差，单位 cm²，越小越可信；
- `UPD` 是两路 ToF 有效更新次数之和。

---

## 7. 已知限制

- 两路正交 ToF 主要修正 x/y，**不能可靠修正长期 yaw 漂移**；航向仍靠陀螺/轮速。
- ToF 测量有 100 ms 左右积分时间和任务延迟，本版未做严格的测量时刻回放；高速时会有厘米级滞后。
- 其他车辆遮挡时，靠“预测墙距 + 新息门控”拒绝，不能保证 100% 区分“贴墙的车辆”和墙。
- 100 ms timing budget 在强光/远距离下量程会下降；需要时按第 5 节调大。
- 本版不包含航点/航向闭环，只输出位姿。
- 默认机械参数（轮径 65 mm、轮距 190 mm、ToF 偏移）是占位值，必须实测修改。

---

## 8. 编译

工程：`MDK-ARM/stm.uvprojx`

新增分组：

- `Application/User/Nav`：mpu6050、tof400、ssd1306、nav_map、nav_vars、nav_ekf、nav_task
- `Drivers/VL53L1X`：ST ULD API + STM32 平台层

包含路径新增：

```text
../Core/Inc/vl53l1x
```

当前编译结果：`0 Error(s), 0 Warning(s)`。

### CubeMX 重新生成注意

1. I2C1 必须配置为 **PB6/PB7、400 kHz**；原 `.ioc` 里是 PB8/PB9、100 kHz，重新生成会覆盖 `i2c.c`。
2. I2C2 必须为 **PB10/PB11、400 kHz**。
3. PB12/PB13 保持 GPIO 输出，作为两片 ToF 的 XSHUT。
4. `Application/User/Nav`、`Drivers/VL53L1X` 分组和 `../Core/Inc/vl53l1x` 包含路径可能被 CubeMX 重写，需要重新加入。
5. `main.c`、`usart.c`、`stm32f1xx_it.c` 的 USER CODE 区会被保留；SysTick 里的 100 Hz 控制调用属于 USER CODE。
6. 本工程当前仍按 STM32F103VD/384 KB Flash 的 Keil 设备配置，但代码约 30 KB，实际 VET6 的 512 KB Flash 完全够用；如需在 Keil 里改成 VET6，只影响器件描述和 Flash 下载范围。
