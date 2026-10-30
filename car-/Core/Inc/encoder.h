/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   四路增量编码器测速接口。
  *
  * 编码器由 TIM2/TIM3/TIM4/TIM5 的编码器接口硬件计数，MCU 只读取 16 位
  * 计数器差值，不占用 CPU 做边沿中断。
  ******************************************************************************
  */
#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 启动四个编码器定时器，并强制使用 4 倍频、清 TIM2 的单脉冲位。 */
void encoder_init(void);

/* 将四个计数器清零。 */
void encoder_reset(void);

/**
  * @brief  读取某个轮子自上次调用以来的编码器增量。
  * @param  wheel 0~3，对应 M1~M4
  * @retval 已应用方向系数的有符号增量（16 位环计数自动展开）
  */
int16_t encoder_read_delta(uint8_t wheel);

/**
  * @brief  编码器增量换算为轮端 rpm。
  * @param  delta  有符号编码器增量
  * @param  dt_s   采样时间，单位秒
  */
float encoder_delta_to_rpm(int16_t delta, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
