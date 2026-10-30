/**
  ******************************************************************************
  * @file    chassis_control.h
  * @brief   四轮差速底盘速度环总入口。
  ******************************************************************************
  */
#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 初始化变量仓库、电机、编码器和 PID。 */
void chassis_control_init(void);

/* 放在 main 的 while(1) 中反复调用，内部按 10 ms 周期执行。 */
void chassis_control_update(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_CONTROL_H */
