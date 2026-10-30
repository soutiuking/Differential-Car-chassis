# 4×4 小车固件仓库

本仓库同时管理 STM32 底盘/定位固件和 ESP32-S3 网页上位机。

```text
car-/                 STM32F103VET6 固件（Keil MDK）
car_up_tools/         ESP32-S3 网页上位机（ESP-IDF）
docs/                 协议、定位、按键/LED、构建指南
```

## 快速入口

- [构建指南](docs/BUILD_GUIDE.md)
- [串口控制协议](car-/docs/CONTROL_PROTOCOL.md)
- [工程总览：按键/LED/OLED/调试窗口](car-/docs/PROJECT_OVERVIEW.md)
- [定位与 EKF](car-/docs/NAVIGATION.md)
- [底盘控制](car-/docs/CHASSIS_CONTROL.md)
- [ESP32 网页上位机](car_up_tools/README.md)

## 当前硬件状态

- 尚未打 PCB，没有为磁力计预留接口。
- 当前固件不包含磁力计驱动/绝对航向融合；MPU6050 仅提供 6 轴数据。
- 现有接口：
  - I2C1 PB6/PB7：MPU6050（0x69，AD0 高）+ SSD1306 OLED（0x3C）
  - I2C2 PB10/PB11：两片 TOF400C（0x60/0x62）
  - USART1 PA9/PA10：ESP32-S3（GPIO17/18）
  - PC0–PC3：4 路共阳极 LED
  - PC4–PC7：4 路用户按键
- K3 标定的是 MPU6050 陀螺零偏，不是磁偏/绝对航向。

## 固件

- [STM32 hex](car-/MDK-ARM/stm/stm.hex)
- [ESP32 bin](car_up_tools/build/car_up_tools.bin)

## Git 约定

- 主分支 `main`
- 不提交 `car-/MDK-ARM/stm/`、`car_up_tools/build/`、IDE 用户文件
- 协议或帧格式变化时，同步更新 `car-/docs/CONTROL_PROTOCOL.md` 和网页端
- 发布时打 tag，例如：

```powershell
git tag -a v0.1.0 -m "底盘 + ESP32 网页上位机"
git push origin main --tags
```
