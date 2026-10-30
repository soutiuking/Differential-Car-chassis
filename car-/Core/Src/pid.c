/**
  ******************************************************************************
  * @file    pid.c
  * @brief   单轮速度环 PID 实现。
  ******************************************************************************
  */
#include "pid.h"
#include "chassis_config.h"
#include <stddef.h>

static float pid_clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

void pid_init(pid_ctrl_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = CHASSIS_PID_KP_DEFAULT;
    pid->ki = CHASSIS_PID_KI_DEFAULT;
    pid->kd = CHASSIS_PID_KD_DEFAULT;
    pid_reset(pid);
}

void pid_reset(pid_ctrl_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_measured = 0.0f;
    pid->d_filtered = 0.0f;
    pid->has_prev = false;
}

void pid_configure(pid_ctrl_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

float pid_update(pid_ctrl_t *pid,
                 float target_rpm,
                 float measured_rpm,
                 float feedforward,
                 float dt_s)
{
    float error;
    float proportional;
    float derivative;
    float new_integral;
    float output;
    float alpha;
    float raw_derivative;

    if ((pid == NULL) || (dt_s <= 0.0f))
    {
        return 0.0f;
    }

    error = target_rpm - measured_rpm;
    proportional = pid->kp * error;

    if (pid->has_prev)
    {
        /* 对测量值求导并一阶低通，目标突变不会产生微分冲击。 */
        raw_derivative = -(measured_rpm - pid->prev_measured) / dt_s;
        alpha = dt_s / (CHASSIS_PID_D_FILTER_TAU_S + dt_s);
        pid->d_filtered += alpha * (raw_derivative - pid->d_filtered);
        derivative = pid->kd * pid->d_filtered;
    }
    else
    {
        derivative = 0.0f;
    }

    new_integral = pid->integral + (pid->ki * error * dt_s);

    output = feedforward + proportional + new_integral + derivative;

    /* 饱和时不再朝饱和方向累积积分。 */
    if ((output > (float)CHASSIS_PWM_MAX) && (error > 0.0f))
    {
        new_integral = pid->integral;
    }
    else if ((output < -(float)CHASSIS_PWM_MAX) && (error < 0.0f))
    {
        new_integral = pid->integral;
    }

    new_integral = pid_clamp(new_integral,
                             -CHASSIS_PID_I_LIMIT,
                             CHASSIS_PID_I_LIMIT);
    pid->integral = new_integral;

    output = feedforward + proportional + new_integral + derivative;
    output = pid_clamp(output,
                       -(float)CHASSIS_PWM_MAX,
                       (float)CHASSIS_PWM_MAX);

    pid->prev_measured = measured_rpm;
    pid->has_prev = true;

    return output;
}
