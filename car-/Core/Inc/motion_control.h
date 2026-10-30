/**
  ******************************************************************************
  * @file    motion_control.h
  * @brief   STM32 端 LINE/ARC/TURN 图元执行器。
  *
  * ESP32 负责把网页上的矩形、圆、手绘折线拆成这些图元并下发；STM32 按队列
  * 逐个执行，回传当前段、横向误差和完成数量。
  ******************************************************************************
  */
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MOTION_IDLE = 0,
    MOTION_RUNNING = 1,
    MOTION_PAUSED = 2,
    MOTION_ERROR = 3
} motion_state_t;

typedef struct
{
    uint8_t state;
    uint8_t active_segment;
    uint8_t total_segments;
    uint8_t completed_segments;
    uint8_t building;
    uint8_t last_error;
    float target_x_m;
    float target_y_m;
    float cross_track_m;
    float heading_error_rad;
    float command_v_mps;
    float command_w_radps;
    float max_speed_mps;
    float max_omega_radps;
} motion_status_t;

void motion_control_init(void);
void motion_control_update(uint32_t now_ms);

bool motion_path_begin(uint8_t expected_segments, uint32_t now_ms);
bool motion_line_to(float x_m, float y_m, float speed_mps);
bool motion_turn_to(float theta_rad, float omega_radps);
bool motion_arc(float radius_m, float sweep_rad, float speed_mps);
bool motion_path_commit(uint32_t now_ms);
void motion_abort(uint32_t now_ms);

void motion_set_limits(float max_speed_mps, float max_omega_radps);
void motion_get_status(motion_status_t *out);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_CONTROL_H */
