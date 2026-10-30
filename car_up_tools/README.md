# ESP32-S3 网页上位机

AP 模式本地网页，通过 UART1 和 STM32 通信。

## 1. 接线

| ESP32-S3 | STM32 | 说明 |
|---|---|---|
| GPIO17 (TX) | PA10 (USART1 RX) | 交叉连接 |
| GPIO18 (RX) | PA9 (USART1 TX) | 交叉连接 |
| GND | GND | 必须共地 |

串口参数：115200 8N1，无流控。

> STM32 的 USART1 原来用于 USB-TTL 调试；接 ESP32 后不要再同时接 USB-TTL 的 TX，避免总线冲突。

## 2. 编译

环境：ESP-IDF v5.4.4，目标 ESP32-S3。

```powershell
& 'C:\Espressif\tools\Microsoft.v5.4.4.PowerShell_profile.ps1'
cd D:\guangdian\car_up_tools
idf.py build
idf.py -p COMx flash monitor
```

当前构建结果：

```text
car_up_tools.bin 约 0xD38D0 字节
```

## 3. 连接网页

上电后 ESP32-S3 建立 AP：

```text
SSID:     Crawler-Car
Password: 12345678
URL:      http://192.168.4.1/
```

手机/电脑连接该 AP 后打开浏览器即可。

## 4. 网页功能

- 4 m×4 m 场地、箭头表示位置/朝向
- 实际轨迹、规划路径、目标点
- 日常测试 / 比赛模式
- 使能、断使能、急停
- 虚拟摇杆
- 点击标点走直线
- 直线、矩形、圆、自由手绘折线
- 速度、PID、参数读写
- ToF 多点标定（开始→采样→拟合→保存）
- x/y、ToF 曲线
- 页面底部 Debug / 错误窗口：连接、HTTP、命令、故障、运动状态、参数/标定错误

## 5. 文件结构

```text
main/
  main.c           初始化
  car_state.c/h    遥测缓存、轨迹、规划点
  uart_link.c/h    UART1 收发、帧解析
  protocol.c/h     二进制帧 CRC/组帧
  path_planner.c/h 形状拆解、路径下发、轨迹合法性监视
  web_server.c/h   AP、HTTP、JSON API
  www/             index.html、app.js、style.css
```

## 6. 注意

- 网页是 AP 本地页面，不依赖互联网。
- 摇杆松开自动发零速；急停会依次发 PATH_ABORT、NAV_STOP、STOP。
- 手绘/圆形路径由 ESP32 拆成小直线段；STM32 的 ARC 图元保留给后续高精度圆弧。
- 参数保存要求底盘已断使能，否则 STM32 拒绝写 Flash。
- ToF 标定至少需要两个已知距离点；建议 300 mm 和 1500 mm。
- “备份到 ESP32”会把网页最近读到的 7 个参数写入 ESP32 NVS；“从 ESP32 恢复”会逐条下发，之后仍需在 STM32 断使能状态下点“保存到 STM32”。
- 原地转向的角度是相对当前航向的角度（度），网页会换算成绝对目标角后下发。
