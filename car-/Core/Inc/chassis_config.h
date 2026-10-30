/**
  ******************************************************************************
  * @file    chassis_config.h
  * @brief   四驱差速底盘集中配置。
  *
  * 说明：
  *  - 本文件只放编译期常量，不保存运行期变量。
  *  - 运行期变量统一放在 chassis_vars.c，通过 get_/give_ 接口访问。
  *  - 机械参数必须按实际底盘测量后修改，否则速度闭环和里程换算不准。
  ******************************************************************************
  */
#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------*/
/* 轮子编号：与接线图 M1 ~ M4 对应                                              */
/* M1 / M3 为左侧，M2 / M4 为右侧（若实际机械布局不同，修改控制层的映射即可）   */
/*---------------------------------------------------------------------------*/
#define CHASSIS_WHEEL_COUNT          4U
#define CHASSIS_WHEEL_M1             0U
#define CHASSIS_WHEEL_M2             1U
#define CHASSIS_WHEEL_M3             2U
#define CHASSIS_WHEEL_M4             3U

/*---------------------------------------------------------------------------*/
/* 控制周期与通信                                                              */
/*---------------------------------------------------------------------------*/
#define CHASSIS_CONTROL_PERIOD_MS    10U      /* 100 Hz 速度环                  */
#define CHASSIS_TELEMETRY_PERIOD_MS  100U     /* 10 Hz 状态回传                 */
#define CHASSIS_UART_BAUDRATE        115200U  /* 与 usart.c 中 USART1 一致      */

#define CHASSIS_COMMAND_TIMEOUT_DEFAULT_MS 300U
#define CHASSIS_COMMAND_TIMEOUT_MIN_MS     50U
#define CHASSIS_COMMAND_TIMEOUT_MAX_MS     2000U

/*---------------------------------------------------------------------------*/
/* 底盘运动学参数（单位：m / rad）                                              */
/*---------------------------------------------------------------------------*/
#define CHASSIS_WHEEL_DIAMETER_M     0.065f   /* 轮子直径，按实测修改            */
#define CHASSIS_TRACK_WIDTH_M        0.190f   /* 左右轮中心距，按实测修改        */
#define CHASSIS_MAX_LINEAR_MPS       0.80f    /* 线速度命令上限                  */
#define CHASSIS_MAX_ANGULAR_RADPS    2.50f    /* 角速度命令上限                  */

#define CHASSIS_PI_F                 3.14159265f

/*---------------------------------------------------------------------------*/
/* 编码器与减速比                                                              */
/* 编码器按“电机轴”每转脉冲数填写 PPR；若厂家给的是减速后输出轴脉冲数，          */
/* 则把 GEARBOX_RATIO 设为 1.0。                                                */
/* 编码器初始化会强制为 4 倍频（TI1 和 TI2 都计数）。                           */
/*---------------------------------------------------------------------------*/
#define ENCODER_PPR                  13.0f
#define ENCODER_EDGES_PER_PULSE      4.0f
#define GEARBOX_RATIO                30.0f
#define CHASSIS_COUNTS_PER_WHEEL_REV \
        (ENCODER_PPR * ENCODER_EDGES_PER_PULSE * GEARBOX_RATIO)

#define CHASSIS_MAX_WHEEL_RPM        400.0f   /* 轮端转速命令上限                */
#define CHASSIS_MAX_MEASURED_RPM     1200.0f  /* 测速合理范围，超出判为异常      */
#define CHASSIS_SENSOR_FAULT_CYCLES  10U      /* 连续异常周期数后判传感器故障    */
#define CHASSIS_ZERO_HOLD_RPM        5.0f     /* 目标为 0 时的防蠕动门限          */

/* 电机/编码器方向：+1 表示正命令驱动底盘前进且编码器正计数，-1 取反。
 * 装车后若某个轮前进方向相反，只改对应的一项，不要改接线。 */
#define CHASSIS_MOTOR_SIGN_M1        1
#define CHASSIS_MOTOR_SIGN_M2        1
#define CHASSIS_MOTOR_SIGN_M3        1
#define CHASSIS_MOTOR_SIGN_M4        1

#define CHASSIS_ENCODER_SIGN_M1      1
#define CHASSIS_ENCODER_SIGN_M2      1
#define CHASSIS_ENCODER_SIGN_M3      1
#define CHASSIS_ENCODER_SIGN_M4      1

/*---------------------------------------------------------------------------*/
/* TB6612 输出与保护                                                           */
/*---------------------------------------------------------------------------*/
#define CHASSIS_PWM_MAX              1000     /* 内部占空比单位：千分比           */
#define CHASSIS_MOTOR_SLEW_PERMILLE  120      /* 每个控制周期的最大变化量         */
#define CHASSIS_MOTOR_REVERSE_DEADTIME_MS 25U /* 换向前先滑行，避免反向电流冲击   */
/* TIM1: 64 MHz / (3199 + 1) = 20 kHz，避开可闻噪声频段。
 * 该值同时用于 motor_init() 的运行时兜底，CubeMX 重新生成后仍会生效。 */
#define CHASSIS_MOTOR_PWM_ARR        3199U    /* TIM1 ARR，20 kHz                 */

/*---------------------------------------------------------------------------*/
/* 速度环 PID 默认参数                                                         */
/* 输出单位：千分比；输入单位：rpm。                                            */
/* Kp: 千分比/rpm，Ki: 千分比/(rpm*s)，Kd: 千分比/(rpm/s)                       */
/*---------------------------------------------------------------------------*/
#define CHASSIS_PID_KP_DEFAULT       1.20f
#define CHASSIS_PID_KI_DEFAULT       0.60f
#define CHASSIS_PID_KD_DEFAULT       0.02f
#define CHASSIS_PID_KP_MAX           10.0f
#define CHASSIS_PID_KI_MAX           20.0f
#define CHASSIS_PID_KD_MAX           5.0f
#define CHASSIS_PID_I_LIMIT          500.0f
#define CHASSIS_PID_D_FILTER_TAU_S   0.020f
#define CHASSIS_FEEDFORWARD_PERMILLE_PER_RPM 2.00f

/*---------------------------------------------------------------------------*/
/* 堵转保护                                                                    */
/*---------------------------------------------------------------------------*/
#define CHASSIS_STALL_PWM_MIN        250
#define CHASSIS_STALL_TARGET_RPM     30.0f
#define CHASSIS_STALL_MEASURED_RPM   8.0f
#define CHASSIS_STALL_TIME_MS        700U

/*---------------------------------------------------------------------------*/
/* 串口帧协议                                                                  */
/* 帧：AA 55 LEN SEQ TYPE PAYLOAD[LEN] CRC16_L CRC16_H                        */
/* CRC16/MODBUS，计算范围：LEN、SEQ、TYPE、PAYLOAD。                           */
/*---------------------------------------------------------------------------*/
#define CHASSIS_PROTOCOL_VERSION     1U
#define CHASSIS_FRAME_SOF1           0xAAU
#define CHASSIS_FRAME_SOF2           0x55U
#define CHASSIS_FRAME_MAX_PAYLOAD    48U
#define CHASSIS_UART_RX_RING_SIZE    512U

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_CONFIG_H */
