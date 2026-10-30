#pragma once

#include <stdbool.h>
#include <stdint.h>

void planner_init(void);
bool planner_goto(float x, float y, float speed_mps);
bool planner_line(const float *x, const float *y, uint16_t count, float speed_mps);
bool planner_rect(float x1, float y1, float x2, float y2, float speed_mps);
bool planner_circle(float cx, float cy, float radius, bool ccw, float speed_mps);
bool planner_polyline(const float *x, const float *y, uint16_t count, float speed_mps);
bool planner_send(void);
void planner_clear(void);
void planner_monitor_task(void *arg);
