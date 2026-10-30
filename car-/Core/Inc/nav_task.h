/**
  ******************************************************************************
  * @file    nav_task.h
  * @brief   导航任务：MPU6050、轮速、双 ToF、EKF、OLED。
  ******************************************************************************
  */
#ifndef NAV_TASK_H
#define NAV_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    NAV_DISPLAY_NORMAL = 0,
    NAV_DISPLAY_DEBUG = 1,
    NAV_DISPLAY_CALIB = 2
} nav_display_mode_t;

void nav_task_init(void);
void nav_task_initial_pose(void);
void nav_task_update(uint32_t now_ms);

void nav_display_set_mode(nav_display_mode_t mode);
nav_display_mode_t nav_display_get_mode(void);
void nav_display_refresh_now(void);

#ifdef __cplusplus
}
#endif

#endif /* NAV_TASK_H */
