/**
  ******************************************************************************
  * @file    motor_tb6612.h
  * @brief   TB6612 四路电机底层操作函数。
  *
  * 本层只负责“某个电机如何转”，不参与 PID 和协议。
  * 所有带 now_ms 的接口用于换向死区计时，调用周期应保持稳定。
  ******************************************************************************
  */
#ifndef MOTOR_TB6612_H
#define MOTOR_TB6612_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 初始化四个通道的 PWM 与方向脚。 */
void motor_init(void);

/* 独立操作函数：直接控制一个电机，占空比单位千分比 0~1000。 */
void motor_coast(uint8_t motor);
void motor_brake(uint8_t motor);
void motor_forward(uint8_t motor, uint16_t permille);
void motor_reverse(uint8_t motor, uint16_t permille);

/**
  * @brief  带符号输出的安全接口，供速度环调用。
  * @param  motor    0~3，对应 M1~M4
  * @param  permille -1000~1000，正数表示底盘前进
  * @param  now_ms   当前毫秒时基
  * @retval 实际施加的带符号占空比；换向死区或滑行期间返回 0
  */
int16_t motor_set_output(uint8_t motor, int16_t permille, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_TB6612_H */
