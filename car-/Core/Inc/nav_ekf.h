/**
  ******************************************************************************
  * @file    nav_ekf.h
  * @brief   6 状态平面 EKF：[x, y, theta, v, omega, gyro_bias]。
  ******************************************************************************
  */
#ifndef NAV_EKF_H
#define NAV_EKF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    NAV_TOF_ACCEPT = 0,
    NAV_TOF_REJECT = 1,
    NAV_TOF_OBSTACLE = 2,
    NAV_TOF_NO_MODEL = 3
} nav_tof_result_t;

void nav_ekf_init(uint32_t now_ms);
void nav_ekf_set_pose(float x_m, float y_m, float theta_rad);
void nav_ekf_set_gyro_bias(float bias_radps);

void nav_ekf_predict(float dt_s);
void nav_ekf_update_wheel(float v_mps, float omega_radps);
void nav_ekf_update_gyro(float gyro_z_radps);
nav_tof_result_t nav_ekf_update_tof(uint8_t sensor, float range_m);

void nav_ekf_get_state(float state[6], float p_diag[6]);

#ifdef __cplusplus
}
#endif

#endif /* NAV_EKF_H */
