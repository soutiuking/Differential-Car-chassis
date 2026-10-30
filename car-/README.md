# 四电机差速底盘 + 4 m×4 m 场地定位固件

基于 `xiaoc.docx` 接线表实现的 STM32F103VET6 + TB6612 四路电机速度闭环，以及 MPU6050 + 双 TOF400C + 轮速的 2D 组合导航。

## 快速入口

- 电机、变量管理、底盘串口协议、PID：[docs/CHASSIS_CONTROL.md](docs/CHASSIS_CONTROL.md)
- 4 m×4 m 定位、双 ToF、EKF、OLED、`0x82` 定位帧：[docs/NAVIGATION.md](docs/NAVIGATION.md)
- 全部串口控制帧、路径图元、参数和标定帧：[docs/CONTROL_PROTOCOL.md](docs/CONTROL_PROTOCOL.md)
- 按键/LED/OLED/调试窗口/完成效果：[docs/PROJECT_OVERVIEW.md](docs/PROJECT_OVERVIEW.md)
- ESP32-S3 AP 网页上位机：[../car_up_tools/README.md](../car_up_tools/README.md)
- 构建/烧录/验证顺序：[../docs/BUILD_GUIDE.md](../docs/BUILD_GUIDE.md)
- 第三方代码许可：[docs/THIRD_PARTY.md](docs/THIRD_PARTY.md)
- Keil 工程：`MDK-ARM/stm.uvprojx`
- 上位机调试脚本：`tools/chassis_host.py`

## 编译

用 Keil MDK 打开 `MDK-ARM/stm.uvprojx`，或命令行：

```bat
"C:\Keil_v5\UV4\UV4.exe" -b MDK-ARM\stm.uvprojx -j0 -o build.log
```

当前版本编译结果：`0 Error(s), 0 Warning(s)`。

## 最小联调流程

```text
1. 115200 8N1 连接 USART1（PA9/PA10）。
2. 发使能帧：AA 55 02 00 04 01 01 DC 51
3. 10~20 Hz 重发速度帧，例如 0.25 m/s：
   AA 55 06 00 01 FA 00 00 00 2C 01 F8 CA
4. 用 tools/chassis_host.py monitor 查看四轮目标/实测/PWM。
5. 需要停止时发停止帧或让命令超时。
```

运动命令默认 300 ms 超时；超时后自动断使能并置故障，必须重新发 `enable=1, clear_faults=1`。

## 目录说明

```text
Core/Inc/chassis_config.h      机械/电气/控制参数
Core/Inc/chassis_vars.h        变量仓库接口（get_/give_/gave_）
Core/Src/chassis_vars.c        变量唯一存储与校验
Core/Src/motor_tb6612.c        TB6612 独立操作
Core/Src/encoder.c             四路编码器测速
Core/Src/pid.c                 单轮速度 PID
Core/Src/chassis_control.c     100 Hz 速度环与安全保护
Core/Src/chassis_protocol.c    USART1 帧协议（0x81 底盘 / 0x82 定位）
Core/Src/mpu6050.c             MPU6050 驱动
Core/Src/tof400.c              双 TOF400C / VL53L1X
Core/Src/ssd1306.c             0.96 寸 OLED
Core/Src/nav_ekf.c             6 状态 EKF
Core/Src/nav_map.c             4 m×4 m 墙线模型
Core/Src/nav_task.c            传感器采集、融合、显示
Core/Src/motion_control.c      LINE/ARC/TURN 图元执行器
Core/Src/param_store.c         Flash 参数/标定持久化
```
