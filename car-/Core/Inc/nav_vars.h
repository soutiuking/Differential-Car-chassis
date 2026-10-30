/**
  ******************************************************************************
  * @file    nav_vars.h
  * @brief   导航输出变量的统一仓库。
  ******************************************************************************
  */
#ifndef NAV_VARS_H
#define NAV_VARS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "chassis_config.h"
#include "chassis_vars.h"

typedef struct
{
    bool ekf_ok;
    bool imu_ok;
    uint8_t mode;          /* 0=日常测试（ToF 不融合）, 1=比赛（ToF 融合） */
    float x_m;
    float y_m;
    float theta_rad;
    float v_mps;
    float omega_radps;
    float gyro_bias_radps;
    float p_xx;
    float p_yy;
    float p_tt;
    float roll_rad;
    float pitch_rad;
    uint16_t tof_mm[2];
    uint8_t tof_status[2];
    bool tof_valid[2];
    bool tof_obstacle[2];
    uint32_t tof_updates[2];
    uint32_t last_update_ms;
} nav_snapshot_t;

void nav_vars_init(void);
void get_nav_snapshot(nav_snapshot_t *out);
var_status_t nav_give_snapshot(const nav_snapshot_t *value);
uint8_t get_nav_mode(void);
var_status_t give_nav_mode(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif /* NAV_VARS_H */
