#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define PLAN_MAX_POINTS   64
#define TRAIL_MAX_POINTS  240

typedef struct {
    bool have_chassis;
    uint16_t faults;
    uint16_t flags;
    int16_t linear_mm_s;
    int16_t angular_mrad_s;
    int16_t target_rpm[4];
    int16_t measured_rpm[4];
    int16_t pwm[4];

    bool have_nav;
    uint8_t nav_mode;
    float x_m;
    float y_m;
    float theta_rad;
    float v_mps;
    float omega_radps;
    float roll_rad;
    float pitch_rad;
    uint16_t tof_mm[2];
    bool tof_valid[2];
    bool tof_obstacle[2];
    float p_xx;
    float p_yy;
    float p_tt;

    bool have_motion;
    uint8_t motion_state;
    uint8_t motion_active;
    uint8_t motion_total;
    uint8_t motion_completed;
    uint8_t motion_building;
    uint8_t motion_error;
    float motion_target_x;
    float motion_target_y;
    float motion_cross_track;
    float motion_heading_error;
    float motion_cmd_v;
    float motion_cmd_w;
    float motion_max_v;
    float motion_max_w;

    bool have_param;
    uint8_t param_id;
    uint8_t param_status;
    int32_t param_value;
    bool params_valid[7];
    int32_t params[7];

    bool have_calib;
    uint8_t calib_action;
    uint8_t calib_sensor;
    uint8_t calib_count;
    uint8_t calib_status;
    uint16_t calib_known_mm;
    uint16_t calib_raw_mm;
    float calib_gain;
    float calib_offset;

    uint16_t plan_count;
    float plan_x[PLAN_MAX_POINTS];
    float plan_y[PLAN_MAX_POINTS];

    uint16_t trail_count;
    float trail_x[TRAIL_MAX_POINTS];
    float trail_y[TRAIL_MAX_POINTS];
} car_state_t;

extern car_state_t g_state;
extern SemaphoreHandle_t g_state_mutex;

void state_init(void);
void state_lock(void);
void state_unlock(void);
void state_update_frame(uint8_t type, const uint8_t *payload, uint8_t len);
void state_set_plan(const float *x, const float *y, uint16_t count);
void state_clear_plan(void);
