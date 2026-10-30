# 四电机差速底盘控制说明

本文对应 `car-` 工程中新增的底盘控制代码。目标：STM32F103VET6 通过 USART1 接收数据帧，读取四路编码器，对四个 TB6612 电机通道做独立速度 PID，实现前进、后退、差速转向和精细调速。定位与显示部分见 [NAVIGATION.md](NAVIGATION.md)。

---

## 1. 文件结构

| 文件 | 作用 |
|---|---|
| `Core/Inc/chassis_config.h` | 机械参数、轮径、轮距、编码器线数、减速比、PID 默认值、保护阈值 |
| `Core/Inc/chassis_vars.h` / `Core/Src/chassis_vars.c` | **唯一变量仓库**，只暴露 `get_*` / `give_*`，校验全部在 `give_*` 内 |
| `Core/Inc/motor_tb6612.h` / `Core/Src/motor_tb6612.c` | TB6612 四路独立操作：滑行、制动、正转、反转、带符号安全输出 |
| `Core/Inc/encoder.h` / `Core/Src/encoder.c` | TIM2/3/4/5 硬件编码器接口，16 位计数回绕展开，轮端 rpm 换算 |
| `Core/Inc/pid.h` / `Core/Src/pid.c` | 单轮速度环：前馈 + PI + 测量微分，积分限幅/饱和回退 |
| `Core/Inc/chassis_control.h` / `Core/Src/chassis_control.c` | 100 Hz 速度环、差速运动学、超时/堵转/传感器保护 |
| `Core/Inc/chassis_protocol.h` / `Core/Src/chassis_protocol.c` | USART1 帧解析、命令分发、10 Hz 状态回传、0x82 定位帧 |
| `Core/Inc/nav_config.h` 等 | MPU6050、双 TOF400C、EKF、OLED、场地模型，详见 [NAVIGATION.md](NAVIGATION.md) |
| `tools/chassis_host.py` | 上位机调试脚本（生成帧、发命令、解析 0x81/0x82 状态） |

`main.c` 只在 USER CODE 区加入初始化与主循环调用；`stm32f1xx_it.c` 增加 `USART1_IRQHandler`；`usart.c` 在 USER CODE 区使能 USART1 NVIC。

---

## 2. 接线映射

来自 `xiaoc.docx` 的引脚表：

| 电机 | PWM | 方向 1 | 方向 2 | 编码器 A | 编码器 B | 编码器定时器 |
|---|---|---|---|---|---|---|
| M1 | PE9 / TIM1_CH1 | PA5 (AIN1) | PA4 (AIN2) | PD12 | PD13 | TIM4_CH1/CH2 |
| M2 | PE11 / TIM1_CH2 | PB14 (BIN1) | PB15 (BIN2) | PA15 | PB3 | TIM2_CH1/CH2，部分重映射 |
| M3 | PE13 / TIM1_CH3 | PD9 (AIN1) | PD8 (AIN2) | PA6 | PA7 | TIM3_CH1/CH2 |
| M4 | PE14 / TIM1_CH4 | PD10 (BIN1) | PD11 (BIN2) | PA0 | PA1 | TIM5_CH1/CH2 |

串口：

| 用途 | 引脚 | 外设 |
|---|---|---|
| 上位机控制帧 / 状态回传 | PA9 TX、PA10 RX | USART1，115200 8N1 |

默认使用 USART1。如果上位机实际接在 UART5（PC12/PD2），需要同步修改 `chassis_protocol.c` 中的 `huart1`/`USART1`、`stm32f1xx_it.c` 的 IRQHandler 和 `usart.c` 的 NVIC；建议优先使用默认的 USART1。

说明：

- `PA15/PB3` 默认是 JTAG 引脚，`HAL_MspInit()` 已经执行 `NOJTAG`（保留 SWD），`encoder_init()` 又显式执行一次，SWD 下载不受影响。
- TIM2 在 CubeMX 里配置了 One-Pulse 虚拟模式，会置 `CR1.OPM`，编码器计到一次更新事件后会停。`encoder_init()` 会清掉该位。
- 代码在运行时把四个编码器定时器强制为 **TI1+TI2 四倍频**，不依赖 CubeMX 里 TIM2 的 TI1 设置。
- 编码器输入使用上拉，输入滤波 6 个采样周期。
- **编码器供电/输出电平必须是 3.3 V**。PA0/PA1 不是 5 V 容忍脚，若编码器是 5 V 推挽输出，必须加电平转换或确认模块输出为开漏/3.3 V。
- TB6612 的 `STBY` 必须为高电平（很多模块用跳线帽默认拉高）；`VM`、`GND` 和电机大电流回路按实际电流布线。

---

## 3. 变量管理规则

### 3.1 唯一存储

`chassis_vars.c` 内：

```c
static chassis_snapshot_t s_vars;
```

`static` 使其他文件无法通过 `extern` 直接读写。所有跨模块数据只能走头文件接口。

### 3.2 读取：`get_*`

- `get_chassis_mode()`、`get_chassis_enabled()`、`get_chassis_faults()`
- `get_linear_command_mps()`、`get_angular_command_radps()`、`get_command_timeout_ms()`、`get_last_command_ms()`
- `get_wheel_target_rpm(i)`、`get_wheel_measured_rpm(i)`、`get_wheel_pwm_permille(i)`
- `get_pid_gains(i, &gains)`
- `get_rx_frames_ok()`、`get_rx_frames_bad()`、`get_control_cycles()`
- `get_chassis_snapshot(&snapshot)`：一次临界区拷贝全部字段，避免读到半更新状态

### 3.3 写入：`give_*`

- `give_motion_command(linear, angular, timeout, now)`
- `give_wheel_command(rpm[4], now)`
- `give_resolved_targets(rpm[4])`
- `give_measured_rpm(wheel, rpm)`
- `give_wheel_pwm(wheel, permille)`
- `give_pid_gains(wheel, kp, ki, kd)`
- `give_enable(enable, clear_faults, now)`
- `give_fault(mask)`、`give_clear_faults()`
- `give_comm_result(frame_ok)`、`give_control_cycle()`

### 3.4 校验位置

数值范围、NaN/Inf、轮号、状态条件全部在 `give_*` 内部完成。调用者只根据返回值处理：

| 返回值 | 含义 |
|---|---|
| `VAR_OK` | 写入成功 |
| `VAR_ERR_ARG` | 空指针/轮号越界 |
| `VAR_ERR_RANGE` | 数值超范围或 NaN |
| `VAR_ERR_STATE` | 未使能、存在故障等状态不允许 |

例如 `give_motion_command()` 内部检查：

- 线速度在 `[-0.80, +0.80] m/s`
- 角速度在 `[-2.50, +2.50] rad/s`
- 超时在 `[50, 2000] ms`
- 必须已使能且无故障

`give_measured_rpm()` 对异常测速值会限幅后写入，并返回 `VAR_ERR_RANGE`，由控制层连续多次判断传感器故障；这样单次毛刺不会立即停机。

### 3.5 `gave_*` 兼容入口

需求中写作 `gave_变量`。头文件保留了 `gave_motion_command()` 等 `static inline` 转发函数；它们内部直接调用 `give_*`，**不重复校验**，校验仍然只在 `give_*` 实现中。

---

## 4. 控制结构

### 4.1 周期

- 速度环：`CHASSIS_CONTROL_PERIOD_MS = 10`，即 100 Hz；由 1 kHz SysTick 分频调用，OLED/传感器阻塞不会拖慢控制。
- 状态回传：`CHASSIS_TELEMETRY_PERIOD_MS = 100`，即 10 Hz。
- PID 使用实际测得的时间差，不假设每次都是精确 10 ms。

### 4.2 差速运动学

输入线速度 `v`（m/s）和角速度 `ω`（rad/s，逆时针为正）：

```text
v_left  = v - ω * track / 2
v_right = v + ω * track / 2
rpm     = v * 60 / (π * wheel_diameter)
```

轮子分配（默认）：

- M1、M3 = 左轮
- M2、M4 = 右轮

如果实际机械布局不是这个顺序，修改 `chassis_control.c` 中四个 `target_rpm[...]` 的左右赋值即可。

### 4.3 单轮速度环

```text
error      = target_rpm - measured_rpm
P          = Kp * error
I          = I + Ki * error * dt
D          = Kd * d(-measured_rpm)/dt，经一阶低通
output     = feedforward + P + I + D
feedforward = target_rpm * 2.0‰/rpm
```

- 输出限幅 `[-1000, +1000]` 千分比。
- 积分限幅 `±500`，输出饱和时回退积分，防止 windup。
- 微分作用在测量值上，目标突变不会产生微分冲击。
- 每个轮子一个独立的 `pid_ctrl_t`，四个参数可单独在线修改。

### 4.4 电机输出安全处理

`motor_set_output()` 是速度环唯一使用的输出接口：

1. 输入限幅到 `±1000`。
2. 方向变化时先滑行 `25 ms`，等电流衰减后再反向。
3. 每个控制周期占空比最多变化 `120‰`，软启动/软减速。
4. 返回实际施加的带符号占空比，写入变量仓库供状态回传。

`motor_forward()` / `motor_reverse()` / `motor_brake()` / `motor_coast()` 是独立底层操作函数，可用于单电机测试；量产控制路径只调用 `motor_set_output()`。

TB6612 真值：

| 操作 | IN1 | IN2 | PWM |
|---|---|---|---|
| 正转 | 1 | 0 | 占空比 |
| 反转 | 0 | 1 | 占空比 |
| 短接制动 | 1 | 1 | 100% |
| 滑行 | 0 | 0 | 0 |

---

## 5. 串口帧协议

### 5.1 帧格式

```text
AA 55 | LEN | SEQ | TYPE | PAYLOAD[LEN] | CRC16_L | CRC16_H
```

- 小端。
- `LEN` 为载荷字节数，最大 48。
- CRC16/MODBUS，多项式 `0xA001`，初值 `0xFFFF`，计算范围：`LEN、SEQ、TYPE、PAYLOAD`（不含 `AA 55` 和 CRC 本身）。
- 标准测试向量：`"123456789"` 的 CRC16/MODBUS = `0x4B37`。

### 5.2 上位机命令

| TYPE | 名称 | LEN | 载荷 |
|---|---|---|---|
| `0x01` | 速度命令 | 6 | `i16 linear_mm_s`、`i16 angular_mrad_s`、`u16 timeout_ms` |
| `0x02` | 四轮转速 | 8 | `i16 rpm_x10[4]`，顺序 M1~M4 |
| `0x03` | PID 参数 | 8 | `u8 wheel`（0~3 或 `0xFF` 全部）、`u8 reserved=0`、`i16 kp_x1000`、`i16 ki_x1000`、`i16 kd_x1000` |
| `0x04` | 使能 | 2 | `u8 enable`（0/1）、`u8 clear_faults`（0/1） |
| `0x05` | 心跳 | 0 | 无，立即回一帧状态 |
| `0x06` | 停止 | 0 | 无，退出使能并清目标 |

约定：

- `timeout_ms = 0` 时使用默认 300 ms。
- 速度命令的 `linear_mm_s` 以 mm/s 为单位，`angular_mrad_s` 以 mrad/s 为单位。
- 四轮转速命令以 0.1 rpm 为单位，例如 `1000` 表示 `100.0 rpm`。
- PID 参数以 0.001 为单位，例如 `1200` 表示 `Kp = 1.2`。
- 必须先发 `0x04 enable=1 clear_faults=1`，再发运动命令。
- 运动命令需要在超时时间内持续发送。建议 10~20 Hz 重发；超时后底盘自动断使能并置 `FAULT_CMD_TIMEOUT`。
- 故障后必须发 `enable=1, clear_faults=1` 才能恢复。

### 5.3 状态回传 `0x81`

每 100 ms 一帧，TYPE=`0x81`，LEN=44。载荷偏移：

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | u16 | `faults` |
| 2 | u16 | `flags`：bit0 enabled，bit1~2 mode（0 IDLE / 1 VEL / 2 WHEEL），bit8~15 协议版本 |
| 4 | i16 | 当前线速度命令，mm/s |
| 6 | i16 | 当前角速度命令，mrad/s |
| 8 | i16 | 距离上一条运动命令的时间，ms，饱和到 32767 |
| 10 | i16[4] | 目标轮速 ×10，rpm，M1~M4 |
| 18 | i16[4] | 实测轮速 ×10，rpm，M1~M4 |
| 26 | i16[4] | 实际 PWM 千分比，M1~M4 |
| 34 | u16 | 接收正常帧计数低 16 位 |
| 36 | u16 | 接收错误/丢弃计数低 16 位 |
| 38 | u32 | 控制周期计数 |
| 42 | i8 | 最近一条命令的 `var_status_t`：0 成功，-1 参数错，-2 范围错，-3 状态错 |
| 43 | u8 | 保留，0 |

故障位：

| 位 | 名称 | 含义 |
|---|---|---|
| 0 | `FAULT_CMD_TIMEOUT` | 运动命令超时 |
| 1~4 | `FAULT_STALL_M1~M4` | 对应轮堵转 |
| 5 | `FAULT_SENSOR` | 连续测速异常 |
| 6 | `FAULT_CONFIG` | 目标/参数超出内部允许范围 |

### 5.4 示例帧

由 `tools/chassis_host.py` 生成：

```text
使能并清故障:
AA 55 02 00 04 01 01 DC 51

速度 0.25 m/s，角速度 0，超时 300 ms:
AA 55 06 00 01 FA 00 00 00 2C 01 F8 CA

四轮 100.0 rpm:
AA 55 08 00 02 E8 03 E8 03 E8 03 E8 03 91 83

四轮 PID Kp=1.2 Ki=0.6 Kd=0.02:
AA 55 08 00 03 FF 00 B0 04 58 02 14 00 21 20
```

### 5.5 上位机脚本

```bash
# 只打印帧，不连接硬件
python tools/chassis_host.py velocity 0.25 0.0

# 连接串口
python tools/chassis_host.py --port COM7 enable
python tools/chassis_host.py --port COM7 velocity 0.25 0.0
python tools/chassis_host.py --port COM7 monitor
```

脚本依赖 `pyserial`。`monitor` 会持续解析 `0x81` 状态帧并打印轮速、PWM、故障。

---

## 6. 安全行为

| 条件 | 动作 |
|---|---|
| 上电 | 变量仓库初始化为未使能，四轮滑行 |
| 未使能时收到运动命令 | `VAR_ERR_STATE`，不执行 |
| 运动命令超过 `timeout_ms` 未刷新 | 置 `FAULT_CMD_TIMEOUT`，断使能，滑行 |
| 目标 ≥30 rpm、输出 ≥250‰，但实测 <8 rpm 或反向持续 700 ms | 置对应 `FAULT_STALL_Mx`，断使能，滑行 |
| 连续 10 个控制周期测速值越界 | 置 `FAULT_SENSOR`，断使能，滑行 |
| 差速分解后的目标超过 400 rpm | 置 `FAULT_CONFIG`，断使能 |
| 任何故障 | `give_fault()` 内部立即清目标并断使能 |
| 换向 | 先滑行 25 ms，再反向 |
| 占空比变化 | 每周期最多 120‰ |

> 注意：本代码没有电流采样，堵转保护是“转速/输出”估算，不能替代硬件过流保护。TB6612 的 VM、GND、电机线和大电流回路必须按实际电流处理。

---

## 7. 参数标定

以下参数在 `Core/Inc/chassis_config.h`：

```c
#define CHASSIS_WHEEL_DIAMETER_M   0.065f
#define CHASSIS_TRACK_WIDTH_M      0.190f
#define ENCODER_PPR                13.0f
#define GEARBOX_RATIO              30.0f
#define CHASSIS_MAX_LINEAR_MPS     0.80f
#define CHASSIS_MAX_ANGULAR_RADPS  2.50f
```

标定顺序：

1. **轮径**：卷尺测轮子外径（m），或让车走 1 m 数轮子转数反算。
2. **轮距**：左右轮中心线距离（m）。
3. **编码器线数/减速比**：
   - 如果厂家给的是“电机轴每转脉冲数”，填 `ENCODER_PPR`，`GEARBOX_RATIO` 填减速比。
   - 如果厂家给的是“输出轴每转脉冲数”，把 `GEARBOX_RATIO` 设为 `1.0`，`ENCODER_PPR` 填输出轴每转脉冲数。
   - 代码固定四倍频，所以 `CHASSIS_COUNTS_PER_WHEEL_REV = PPR * 4 * 减速比`。
4. **方向**：
   - 单轮给正命令，若轮子反转，把对应 `CHASSIS_MOTOR_SIGN_Mx` 改成 `-1`。
   - 手转轮子前进方向，若实测 rpm 为负，把对应 `CHASSIS_ENCODER_SIGN_Mx` 改成 `-1`。
   - 两者必须同时为正方向，否则速度环会正反馈。
5. **左右轮分配**：确认 M1/M3 在左、M2/M4 在右；否则修改控制层映射。

> 默认的 13 PPR、30:1、65 mm 轮径只是常见小车的占位值，必须按实际硬件核对。

---

## 8. PID 调参建议

1. 架空底盘，先确认四个轮子方向、编码器符号正确。
2. 使能后给低目标（例如四轮 50 rpm），观察 `measured_rpm` 是否跟随、是否正反馈。
3. 先只用 `Kp`：从 `0.5` 开始增大，直到响应快但略有振荡，再回退 20%~30%。
4. 加 `Ki`：从 `0.1~0.3` 开始，消除稳态误差；积分过大时会低频摆动。
5. 加 `Kd`：默认 `0.02` 很小，只在超调明显时增加；速度环噪声大时不要加太多。
6. 四个轮子机械阻力不同，可以单独用 `0x03` 命令给某个轮不同参数。
7. 调好后把参数写回 `chassis_config.h` 的默认值。

默认前馈 `2.0‰/rpm` 是粗略值；若电机在 200 rpm 只需要 300‰，可把它降到 `1.5`，让 PID 少做工作。

---

## 9. 编译与下载

- 工程：`car-/MDK-ARM/stm.uvprojx`
- 编译器：ARMCC V5.06（Keil MDK）
- 已编译通过：`0 Error(s), 0 Warning(s)`
- 输出：`car-/MDK-ARM/stm/stm.axf`、`stm.hex`

新增源文件已加入 Keil 分组 `Application/User/Chassis`。

### CubeMX 重新生成注意事项

本工程仍可用 CubeMX 维护，但重新生成后请检查：

1. `Application/User/Chassis` 分组和 6 个 `.c` 文件是否还在 Keil 工程里；CubeMX 可能重写 `.uvprojx`。
2. `main.c`、`usart.c`、`stm32f1xx_it.c` 的修改都在 USER CODE 区，正常会保留。
3. `encoder_init()` 会在运行时清 `TIM2` 的 `OPM` 并改四倍频，所以即使 CubeMX 仍配置为 TI1/One-Pulse，功能也正确；但建议在 CubeMX 里删除 TIM2 的 One-Pulse 虚拟模式，避免误解。
4. USART1 NVIC 使能在 `usart.c` 的 USER CODE 区；如果 CubeMX 里也勾选 USART1 中断，注意不要生成重复的 `USART1_IRQHandler`。
5. TIM1 的 ARR 已改为 `3199`（20 kHz）；即使 CubeMX 重新生成回旧值，`motor_init()` 也会用 `CHASSIS_MOTOR_PWM_ARR` 覆盖。
6. 新增文件是手写模块，不在 `.ioc` 中管理。

---

## 10. 已知限制

- 速度环基于编码器，没有 IMU/电流环；打滑时轮速闭环不等于车体速度闭环。
- 状态回传使用阻塞发送，51 字节约 4.4 ms（115200），每 100 ms 一次；对 100 Hz 控制环影响很小，但后续可改 DMA。
- 堵转保护是估算，不是过流保护。
- PWM 频率已提高到 20 kHz（TIM1 ARR=3199，64 MHz），在人耳可闻频段之外，同时保留 3200 级占空比分辨率。`motor_init()` 会用 `CHASSIS_MOTOR_PWM_ARR` 再设置一次，CubeMX 重新生成后也不会退回 977 Hz。
- 默认轮径/轮距/编码器参数是占位值，装车前必须标定。
