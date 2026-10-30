# 串口控制协议

适用对象：

- STM32 USART1，PA9/PA10，115200 8N1
- ESP32-S3 UART1，GPIO17 TX / GPIO18 RX，接 STM32 PA10/PA9，共地
- 浏览器不直接连 STM32，而是通过 ESP32-S3 的网页/HTTP 转成二进制帧

## 1. 通用帧

```text
AA 55 | LEN | SEQ | TYPE | PAYLOAD[LEN] | CRC16_L | CRC16_H
```

- 小端
- `LEN`：载荷字节数，最大 48
- `SEQ`：发送序号，当前 STM32 不校验
- CRC16/MODBUS，计算范围：`LEN、SEQ、TYPE、PAYLOAD`
- 所有多字节整数为小端

## 2. 已有底盘控制帧

| TYPE | 名称 | LEN | 载荷 |
|---|---|---:|---|
| `0x01` | 速度控制 | 6 | `i16 linear_mm_s`、`i16 angular_mrad_s`、`u16 timeout_ms` |
| `0x02` | 四轮转速 | 8 | `i16 rpm_x10[4]` |
| `0x03` | PID 参数 | 8 | `u8 wheel`（0~3 或 0xFF）、`u8 reserved`、`i16 kp_x1000`、`i16 ki_x1000`、`i16 kd_x1000` |
| `0x04` | 使能 | 2 | `u8 enable`、`u8 clear_faults` |
| `0x05` | 心跳 | 0 | 无 |
| `0x06` | 底盘停止 | 0 | 退出使能 |

摇杆控制使用 `0x01`，20 Hz 发送；松手发 `v=0,w=0`。

## 3. 导航/路径控制帧

### `0x10` SET_NAV_MODE

LEN=1：

```text
u8 mode: 0=日常测试（ToF 不参与 EKF）, 1=比赛（ToF 融合）
```

### `0x11` LINE_TO

LEN=8：

```text
i16 target_x_mm
i16 target_y_mm
u16 speed_mm_s
u8  flags       // 保留，填 0
u8  reserved
```

必须在 `PATH_BEGIN` 之后、`PATH_COMMIT` 之前发送。

### `0x12` TURN_TO

LEN=4：

```text
i16 target_theta_mrad
i16 omega_mrad_s
```

### `0x13` ARC

LEN=8：

```text
i16 radius_mm
i16 sweep_mrad      // 正=逆时针，负=顺时针
u16 speed_mm_s
u8  flags
u8  reserved
```

圆弧圆心由 STM32 按当前位姿和 `sweep` 符号自动生成。正 sweep 时圆心在车体左侧，负 sweep 在右侧。

### `0x14` NAV_STOP

LEN=0，停止当前路径，底盘保持使能并发送零速。

### `0x15` SET_SPEED

LEN=4：

```text
u16 max_speed_mm_s
u16 max_omega_mrad_s
```

### `0x16` PATH_BEGIN

LEN=2：

```text
u8 total_segments
u8 flags
```

清空队列并进入“构建路径”状态。

### `0x17` PATH_COMMIT

LEN=0，提交并开始执行。

### `0x18` PATH_ABORT

LEN=0，清空队列、零速、退出路径执行。

### `0x19` CALIB

LEN=6：

```text
u8 action: 0=开始,1=采样,2=拟合并应用,3=保存Flash,4=从Flash加载,5=复位
u8 sensor: 0=前向,1=右向
u16 known_mm
u16 reserved
```

多点标定步骤：

1. `action=0` 清空该传感器采样点；
2. 把小车/挡板放到已知距离 `known_mm`；
3. `action=1` 采样，重复至少两个距离；
4. `action=2` 最小二乘拟合 `corrected = gain*raw + offset`；
5. `action=3` 保存到 STM32 Flash；
6. 需要恢复时 `action=4`；复位 `action=5`。

### `0x1A` PARAM

LEN=6：

```text
u8 action: 0=get,1=set,2=save,3=load,4=reset defaults
u8 param_id
i32 value
```

参数 ID：

| ID | 含义 | 单位 |
|---:|---|---|
| 0 | 最大线速度 | mm/s |
| 1 | 最大角速度 | mrad/s |
| 2 | 导航模式 | 0/1 |
| 3 | 前向 ToF gain | ×1000 |
| 4 | 前向 ToF offset | mm |
| 5 | 右向 ToF gain | ×1000 |
| 6 | 右向 ToF offset | mm |

PID 仍用 `0x03` 设置；`action=2` 保存时会同时把当前四轮 PID 写入 Flash。

STM32 参数页地址为 `0x0805F800`（按 384 KB Flash 的最后一页使用；VET6 的 512 KB 空间内同样有效）。保存要求底盘已断使能，否则返回失败。

## 4. 回传帧

### `0x81` 底盘状态

LEN=44，布局见 `docs/CHASSIS_CONTROL.md`。

### `0x82` 定位状态

LEN=30：

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | u16 | flags：bit0 EKF，bit1 IMU，bit2 前向 ToF 有效，bit3 右向有效，bit4/5 障碍物，bit6 比赛模式 |
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
| 26 | u16 | 前向有效更新次数低 16 位 |
| 28 | u16 | 右向有效更新次数低 16 位 |

### `0x83` 路径控制状态

LEN=28：

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | u8 | motion state：0 IDLE / 1 RUN / 2 PAUSE / 3 ERROR |
| 1 | u8 | active_segment |
| 2 | u8 | total_segments |
| 3 | u8 | completed_segments |
| 4 | u8 | building |
| 5 | u8 | last_error |
| 6 | u8 | flags |
| 7 | u8 | reserved |
| 8 | i16 | target_x，mm |
| 10 | i16 | target_y，mm |
| 12 | i16 | cross_track，mm |
| 14 | i16 | heading_error，mrad |
| 16 | i16 | command_v，mm/s |
| 18 | i16 | command_w，mrad/s |
| 20 | u16 | max_speed，mm/s |
| 22 | u16 | max_omega，mrad/s |
| 24 | u16 | reserved |
| 26 | u16 | reserved |

### `0x84` 参数回读

LEN=8：

```text
u8  param_id
u8  status: 0成功，1失败
i32 value
u16 reserved
```

### `0x85` 标定回读

LEN=16：

```text
u8  action
u8  sensor
u8  point_count
u8  status
u16 known_mm
u16 raw_mm
i16 gain_x1000
i16 offset_mm
u16 reserved
u16 reserved
```

## 5. 典型命令序列

### 5.1 使能 + 摇杆

```text
1. 0x04 [1,1]                 使能并清故障
2. 0x10 [0]                   日常模式
3. 0x01 [v_mm_s, w_mrad_s, 300] 20 Hz 发送
4. 松手：0x01 [0,0,300]
```

### 5.2 点击场地走直线

```text
0x16 [1,0]                    开始，1 段
0x11 [x_mm, y_mm, speed_mm_s, 0, 0]
0x17                          提交
```

示例：走到 (1.5, 2.0)，速度 0.3 m/s：

```text
AA 55 02 00 16 01 00 BD 94
AA 55 08 00 11 DC 05 D0 07 2C 01 00 00 4B DD
AA 55 00 00 17 31 CE
```

### 5.3 手绘折线

ESP32 把折线简化成 N 个点，发送：

```text
0x16 [N,0]
0x11 ...  重复 N 次
0x17
```

### 5.4 矩形/圆

- 矩形：4 条 LINE（首尾闭合）
- 圆：ESP32 拆成 36 条 LINE 近似；STM32 的 ARC 图元保留给后续高精度圆弧
- 仅 STM32 支持的 ARC 可以直接用 `0x13` 发圆弧

### 5.5 停止

```text
0x18 PATH_ABORT   停路径，底盘保持使能
0x14 NAV_STOP     停导航
0x06 STOP         断使能
```

网页急停依次发送三条。

## 6. ESP32 网页接口

AP：

```text
SSID:     Crawler-Car
Password: 12345678
URL:      http://192.168.4.1/
```

HTTP：

```text
GET  /api/state   返回底盘/定位/路径/参数/标定 JSON
POST /api/cmd     发送控制命令
```

`/api/cmd` 示例：

```json
{"cmd":"mode","mode":1}
{"cmd":"enable"}
{"cmd":"velocity","v":0.25,"w":0.0}
{"cmd":"goto","x":1.5,"y":2.0,"speed":0.3}
{"cmd":"turn","theta":1.5708}   // 网页接口按“相对当前航向”解释，ESP32 转成绝对目标角后发 0x12
{"cmd":"line","points":[{"x":1.0,"y":1.0},{"x":2.0,"y":2.0}],"speed":0.25}
{"cmd":"rect","x1":1.0,"y1":1.0,"x2":2.0,"y2":2.0,"speed":0.25}
{"cmd":"circle","cx":2.0,"cy":2.0,"r":0.5,"ccw":1,"speed":0.25}
{"cmd":"polyline","points":[{"x":0.5,"y":0.5},{"x":1.5,"y":1.0}],"speed":0.25}
{"cmd":"stop"}
{"cmd":"nav_stop"}
{"cmd":"set_speed","max_v":0.5,"max_w":1.5}
{"cmd":"pid","wheel":255,"kp":1.2,"ki":0.6,"kd":0.02}
{"cmd":"calib","action":1,"sensor":0,"known":300}
{"cmd":"param","action":1,"id":0,"value":500}
{"cmd":"save"}
{"cmd":"load"}
{"cmd":"backup"}      // 把最近回读的 7 个参数备份到 ESP32 NVS
{"cmd":"restore"}     // 从 ESP32 NVS 逐条下发参数；之后再 save 到 STM32
```
