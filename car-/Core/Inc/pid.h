/**
  ******************************************************************************
  * @file    pid.h
  * @brief   单轮速度环 PID。
  *
  * 采用“前馈 + PI + 测量微分”的结构：
  *  - 前馈让输出快速接近目标；
  *  - 积分消除稳态误差；
  *  - 微分作用在测量值上，避免目标突变造成冲击；
  *  - 积分限幅 + 饱和回退，防止积分饱和。
  *
  * 本模块不校验增益，增益必须经 chassis_vars.c 的 give_pid_gains() 写入。
  ******************************************************************************
  */
#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_measured;
    float d_filtered;
    bool has_prev;
} pid_ctrl_t;

void pid_init(pid_ctrl_t *pid);
void pid_reset(pid_ctrl_t *pid);
void pid_configure(pid_ctrl_t *pid, float kp, float ki, float kd);

/**
  * @brief  计算一次 PID 输出。
  * @param  pid          控制器
  * @param  target_rpm   目标轮速
  * @param  measured_rpm 实测轮速
  * @param  feedforward  前馈输出（千分比）
  * @param  dt_s         实际控制周期，秒
  * @retval 千分比输出，已限幅到 [-1000, 1000]
  */
float pid_update(pid_ctrl_t *pid,
                 float target_rpm,
                 float measured_rpm,
                 float feedforward,
                 float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
