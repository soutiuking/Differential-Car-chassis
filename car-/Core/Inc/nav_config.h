/**
  ******************************************************************************
  * @file    nav_config.h
  * @brief   4 m × 4 m 场地、ToF 安装和导航融合参数。
  ******************************************************************************
  */
#ifndef NAV_CONFIG_H
#define NAV_CONFIG_H

#include "chassis_config.h"

#define NAV_PI_F                      3.14159265f
#define NAV_FIELD_LENGTH_M            4.0f
#define NAV_FIELD_WIDTH_M             4.0f

/* 场地坐标原点在左下角，x 向右、y 向上；上电车头朝 +x。 */
#define NAV_START_X_M                 0.50f
#define NAV_START_Y_M                 0.50f
#define NAV_START_THETA_RAD           0.0f

/* 车体坐标：+x 向前，+y 向左，逆时针为航向正方向。 */
#define NAV_TOF_FRONT_OFFSET_X_M      0.080f
#define NAV_TOF_FRONT_OFFSET_Y_M      0.000f
#define NAV_TOF_FRONT_ANGLE_RAD       0.0f
#define NAV_TOF_RIGHT_OFFSET_X_M      0.000f
#define NAV_TOF_RIGHT_OFFSET_Y_M     -0.060f
#define NAV_TOF_RIGHT_ANGLE_RAD      (-NAV_PI_F * 0.5f)

/* 导航/传感器周期。 */
#define NAV_PERIOD_MS                 10U
#define NAV_MPU_PERIOD_MS             10U
#define NAV_TOF_POLL_PERIOD_MS        10U
#define NAV_DISPLAY_PERIOD_MS         20U
#define NAV_TELEMETRY_PERIOD_MS       100U

/* ToF 输出 10 Hz 左右，两片错开启动；融合更新不需要跟控制环同频。 */
#define NAV_TOF_STABLE_SAMPLES        2U
#define NAV_TOF_INNOVATION_BASE_M     0.10f
#define NAV_TOF_GATE_SIGMA            4.0f
#define NAV_TOF_OBSTACLE_MARGIN_M     0.02f
#define NAV_TOF_MIN_RANGE_M           0.03f
#define NAV_TOF_MAX_RANGE_M           3.95f

/* EKF 噪声参数（可按实车标定）。 */
#define NAV_EKF_Q_POS                 0.000020f
#define NAV_EKF_Q_THETA               0.000002f
#define NAV_EKF_Q_V                   0.000500f
#define NAV_EKF_Q_OMEGA               0.002000f
#define NAV_EKF_Q_BIAS                0.0000000001f
#define NAV_EKF_R_WHEEL_V             0.0400f
#define NAV_EKF_R_WHEEL_OMEGA         0.1200f
#define NAV_EKF_R_GYRO                0.0300f
#define NAV_EKF_R_TOF_BASE            0.0300f
#define NAV_EKF_R_TOF_SCALE           0.0100f

/* 传感器安装方向修正；装反时改符号。 */
#define NAV_GYRO_Z_SIGN               1.0f
#define NAV_WHEEL_LEFT_SIGN           1.0f
#define NAV_WHEEL_RIGHT_SIGN          1.0f

#endif /* NAV_CONFIG_H */
