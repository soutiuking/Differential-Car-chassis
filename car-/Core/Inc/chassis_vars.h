/**
  ******************************************************************************
  * @file    chassis_vars.h
  * @brief   底盘运行期变量的统一仓库。
  *
  * 设计约束：
  *  1. 所有需要跨模块传递的变量只允许由 chassis_vars.c 内的静态结构保存。
  *  2. 读取只能调用 get_*；写入只能调用 give_*。
  *  3. 数值范围、合法性、状态检查全部放在 give_* 内部完成，调用者不再重复校验。
  *  4. 不向任何模块暴露全局变量地址或可写指针。
  ******************************************************************************
  */
#ifndef CHASSIS_VARS_H
#define CHASSIS_VARS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "chassis_config.h"

/*---------------------------------------------------------------------------*/
/* 故障位                                                                      */
/*---------------------------------------------------------------------------*/
#define FAULT_CMD_TIMEOUT    (1U << 0)
#define FAULT_STALL_M1       (1U << 1)
#define FAULT_STALL_M2       (1U << 2)
#define FAULT_STALL_M3       (1U << 3)
#define FAULT_STALL_M4       (1U << 4)
#define FAULT_SENSOR         (1U << 5)
#define FAULT_CONFIG         (1U << 6)

#define FAULT_STALL_MASK     (FAULT_STALL_M1 | FAULT_STALL_M2 | \
                              FAULT_STALL_M3 | FAULT_STALL_M4)
#define FAULT_ALL_MASK       (FAULT_CMD_TIMEOUT | FAULT_STALL_MASK | \
                              FAULT_SENSOR | FAULT_CONFIG)

/*---------------------------------------------------------------------------*/
/* 对外类型                                                                    */
/*---------------------------------------------------------------------------*/
typedef enum
{
    CHASSIS_MODE_IDLE = 0,       /* 已使能但没有有效运动命令 */
    CHASSIS_MODE_VELOCITY = 1,   /* 线速度 + 角速度命令      */
    CHASSIS_MODE_WHEEL_RPM = 2   /* 四轮转速直接命令          */
} chassis_mode_t;

typedef enum
{
    VAR_OK = 0,
    VAR_ERR_ARG = -1,     /* 空指针/非法编号 */
    VAR_ERR_RANGE = -2,   /* 数值超范围/NaN  */
    VAR_ERR_STATE = -3    /* 当前状态不允许   */
} var_status_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
} pid_gains_t;

/* 只在 get_chassis_snapshot() 中整块拷贝，避免读到半更新的一组值。 */
typedef struct
{
    chassis_mode_t mode;
    bool enabled;
    uint16_t faults;
    float linear_mps;
    float angular_radps;
    uint32_t command_timeout_ms;
    uint32_t last_command_ms;
    float target_rpm[CHASSIS_WHEEL_COUNT];
    float measured_rpm[CHASSIS_WHEEL_COUNT];
    int16_t pwm_permille[CHASSIS_WHEEL_COUNT];
    pid_gains_t pid[CHASSIS_WHEEL_COUNT];
    uint32_t rx_frames_ok;
    uint32_t rx_frames_bad;
    uint32_t control_cycles;
} chassis_snapshot_t;

/*---------------------------------------------------------------------------*/
/* 初始化                                                                      */
/*---------------------------------------------------------------------------*/
void vars_init(void);

/*---------------------------------------------------------------------------*/
/* get_*：只读访问                                                             */
/*---------------------------------------------------------------------------*/
chassis_mode_t get_chassis_mode(void);
bool get_chassis_enabled(void);
uint16_t get_chassis_faults(void);

float get_linear_command_mps(void);
float get_angular_command_radps(void);
uint32_t get_command_timeout_ms(void);
uint32_t get_last_command_ms(void);

float get_wheel_target_rpm(uint8_t wheel);
float get_wheel_measured_rpm(uint8_t wheel);
int16_t get_wheel_pwm_permille(uint8_t wheel);
void get_pid_gains(uint8_t wheel, pid_gains_t *out);

uint32_t get_rx_frames_ok(void);
uint32_t get_rx_frames_bad(void);
uint32_t get_control_cycles(void);

/* 一致性快照；out 为空时直接返回。 */
void get_chassis_snapshot(chassis_snapshot_t *out);

/*---------------------------------------------------------------------------*/
/* give_*：唯一允许写入变量仓库的入口，校验全部在函数内部完成                    */
/*---------------------------------------------------------------------------*/
var_status_t give_motion_command(float linear_mps,
                                 float angular_radps,
                                 uint32_t timeout_ms,
                                 uint32_t now_ms);
var_status_t give_wheel_command(const float rpm[CHASSIS_WHEEL_COUNT],
                                uint32_t now_ms);
var_status_t give_resolved_targets(const float rpm[CHASSIS_WHEEL_COUNT]);

var_status_t give_measured_rpm(uint8_t wheel, float rpm);
var_status_t give_wheel_pwm(uint8_t wheel, int16_t permille);
var_status_t give_pid_gains(uint8_t wheel, float kp, float ki, float kd);

var_status_t give_enable(bool enable, bool clear_faults, uint32_t now_ms);
var_status_t give_fault(uint16_t fault_mask);
var_status_t give_clear_faults(void);

var_status_t give_comm_result(bool frame_ok);
var_status_t give_control_cycle(void);

/*---------------------------------------------------------------------------*/
/* 兼容入口：需求中写作 gave_*。                                               */
/* 这些函数只做转发，不重复校验；真正的校验仍只在 give_* 内完成。               */
/*---------------------------------------------------------------------------*/
static inline var_status_t gave_motion_command(float linear_mps,
                                               float angular_radps,
                                               uint32_t timeout_ms,
                                               uint32_t now_ms)
{
    return give_motion_command(linear_mps, angular_radps, timeout_ms, now_ms);
}

static inline var_status_t gave_wheel_command(const float rpm[CHASSIS_WHEEL_COUNT],
                                              uint32_t now_ms)
{
    return give_wheel_command(rpm, now_ms);
}

static inline var_status_t gave_resolved_targets(const float rpm[CHASSIS_WHEEL_COUNT])
{
    return give_resolved_targets(rpm);
}

static inline var_status_t gave_measured_rpm(uint8_t wheel, float rpm)
{
    return give_measured_rpm(wheel, rpm);
}

static inline var_status_t gave_wheel_pwm(uint8_t wheel, int16_t permille)
{
    return give_wheel_pwm(wheel, permille);
}

static inline var_status_t gave_pid_gains(uint8_t wheel, float kp, float ki, float kd)
{
    return give_pid_gains(wheel, kp, ki, kd);
}

static inline var_status_t gave_enable(bool enable, bool clear_faults, uint32_t now_ms)
{
    return give_enable(enable, clear_faults, now_ms);
}

static inline var_status_t gave_fault(uint16_t fault_mask)
{
    return give_fault(fault_mask);
}

static inline var_status_t gave_clear_faults(void)
{
    return give_clear_faults();
}

static inline var_status_t gave_comm_result(bool frame_ok)
{
    return give_comm_result(frame_ok);
}

static inline var_status_t gave_control_cycle(void)
{
    return give_control_cycle();
}

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_VARS_H */
