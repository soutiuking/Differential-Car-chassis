/**
  ******************************************************************************
  * @file    motion_control.c
  * @brief   LINE / TURN / ARC 图元队列执行器。
  ******************************************************************************
  */
#include "motion_control.h"
#include "nav_config.h"
#include "nav_vars.h"
#include "chassis_vars.h"
#include "chassis_config.h"
#include <math.h>
#include <string.h>

#define MOTION_MAX_SEGMENTS   64U

typedef enum
{
    SEG_LINE = 0,
    SEG_TURN = 1,
    SEG_ARC = 2
} motion_seg_type_t;

typedef struct
{
    uint8_t type;
    uint8_t started;
    float x_m;
    float y_m;
    float theta_rad;
    float radius_m;
    float sweep_rad;
    float speed_mps;
    float center_x;
    float center_y;
    float last_angle;
    float accum_angle;
} motion_seg_t;

static motion_seg_t s_seg[MOTION_MAX_SEGMENTS];
static uint8_t s_count;
static uint8_t s_expected;
static uint8_t s_active;
static uint8_t s_completed;
static uint8_t s_state;
static uint8_t s_building;
static uint8_t s_last_error;
static bool s_owns_control;
static uint32_t s_last_update_ms;
static float s_max_speed;
static float s_max_omega;
static float s_cross_track;
static float s_heading_error;
static float s_cmd_v;
static float s_cmd_w;
static float s_target_x;
static float s_target_y;

static float motion_clamp(float value, float lo, float hi)
{
    if (value < lo)
    {
        return lo;
    }
    if (value > hi)
    {
        return hi;
    }
    return value;
}

static float motion_norm_angle(float angle)
{
    while (angle > NAV_PI_F)
    {
        angle -= 2.0f * NAV_PI_F;
    }
    while (angle < -NAV_PI_F)
    {
        angle += 2.0f * NAV_PI_F;
    }
    return angle;
}

static bool motion_finite(float value)
{
    return (value == value) && (value <= 1.0e30f) && (value >= -1.0e30f);
}

static void motion_send(float v_mps, float w_radps, uint32_t now_ms)
{
    var_status_t status;

    s_cmd_v = motion_clamp(v_mps, -s_max_speed, s_max_speed);
    s_cmd_w = motion_clamp(w_radps, -s_max_omega, s_max_omega);

    status = give_motion_command(s_cmd_v, s_cmd_w,
                                 CHASSIS_COMMAND_TIMEOUT_DEFAULT_MS,
                                 now_ms);
    if (status != VAR_OK)
    {
        s_last_error = (uint8_t)((status < 0) ? -status : status);
        s_state = MOTION_ERROR;
        s_owns_control = false;
    }
}

static void motion_complete(uint32_t now_ms)
{
    give_motion_command(0.0f, 0.0f,
                        CHASSIS_COMMAND_TIMEOUT_DEFAULT_MS, now_ms);
    s_state = MOTION_IDLE;
    s_owns_control = false;
    s_cmd_v = 0.0f;
    s_cmd_w = 0.0f;
    s_active = s_count;
    s_completed = s_count;
}

static void motion_fail(uint8_t error_code, uint32_t now_ms)
{
    (void)give_motion_command(0.0f, 0.0f,
                              CHASSIS_COMMAND_TIMEOUT_DEFAULT_MS, now_ms);
    s_last_error = error_code;
    s_state = MOTION_ERROR;
    s_owns_control = false;
    s_cmd_v = 0.0f;
    s_cmd_w = 0.0f;
}

static bool motion_next_segment(uint32_t now_ms)
{
    s_completed++;
    s_active++;
    if (s_active >= s_count)
    {
        motion_complete(now_ms);
        return false;
    }
    s_seg[s_active].started = 0U;
    return true;
}

void motion_control_init(void)
{
    s_count = 0U;
    s_expected = 0U;
    s_active = 0U;
    s_completed = 0U;
    s_state = MOTION_IDLE;
    s_building = 0U;
    s_last_error = 0U;
    s_owns_control = false;
    s_last_update_ms = 0U;
    s_max_speed = 0.60f;
    s_max_omega = 2.00f;
    s_cross_track = 0.0f;
    s_heading_error = 0.0f;
    s_cmd_v = 0.0f;
    s_cmd_w = 0.0f;
    s_target_x = 0.0f;
    s_target_y = 0.0f;
}

bool motion_path_begin(uint8_t expected_segments, uint32_t now_ms)
{
    if (expected_segments > MOTION_MAX_SEGMENTS)
    {
        s_last_error = 2U;
        return false;
    }

    if (s_state == MOTION_RUNNING)
    {
        motion_abort(now_ms);
    }

    s_count = 0U;
    s_expected = expected_segments;
    s_active = 0U;
    s_completed = 0U;
    s_building = 1U;
    s_state = MOTION_IDLE;
    s_last_error = 0U;
    s_owns_control = false;
    return true;
}

static bool motion_enqueue_base(uint8_t type, float speed_mps)
{
    if ((!s_building) || (s_count >= MOTION_MAX_SEGMENTS) ||
        (!motion_finite(speed_mps)))
    {
        s_last_error = 2U;
        return false;
    }

    memset(&s_seg[s_count], 0, sizeof(motion_seg_t));
    s_seg[s_count].type = type;
    s_seg[s_count].speed_mps = motion_clamp(speed_mps, 0.03f, s_max_speed);
    s_count++;
    return true;
}

bool motion_line_to(float x_m, float y_m, float speed_mps)
{
    if ((!motion_finite(x_m)) || (!motion_finite(y_m)) ||
        (x_m < -1.0f) || (x_m > (NAV_FIELD_LENGTH_M + 1.0f)) ||
        (y_m < -1.0f) || (y_m > (NAV_FIELD_WIDTH_M + 1.0f)))
    {
        s_last_error = 3U;
        return false;
    }
    if (!motion_enqueue_base(SEG_LINE, speed_mps))
    {
        return false;
    }
    s_seg[s_count - 1U].x_m = x_m;
    s_seg[s_count - 1U].y_m = y_m;
    return true;
}

bool motion_turn_to(float theta_rad, float omega_radps)
{
    if ((!motion_finite(theta_rad)) || (!motion_finite(omega_radps)))
    {
        s_last_error = 4U;
        return false;
    }
    if (!motion_enqueue_base(SEG_TURN, 0.03f))
    {
        return false;
    }
    s_seg[s_count - 1U].theta_rad = motion_norm_angle(theta_rad);
    s_seg[s_count - 1U].speed_mps = motion_clamp(omega_radps, 0.05f, s_max_omega);
    return true;
}

bool motion_arc(float radius_m, float sweep_rad, float speed_mps)
{
    if ((!motion_finite(radius_m)) || (!motion_finite(sweep_rad)) ||
        (radius_m < 0.05f) || (radius_m > 20.0f) ||
        (fabsf(sweep_rad) < 1.0e-4f) || (fabsf(sweep_rad) > (2.0f * NAV_PI_F + 0.01f)))
    {
        s_last_error = 4U;
        return false;
    }
    if (!motion_enqueue_base(SEG_ARC, speed_mps))
    {
        return false;
    }
    s_seg[s_count - 1U].radius_m = radius_m;
    s_seg[s_count - 1U].sweep_rad = sweep_rad;
    return true;
}

bool motion_path_commit(uint32_t now_ms)
{
    if ((!s_building) || (s_count == 0U))
    {
        s_last_error = 5U;
        return false;
    }

    s_building = 0U;
    s_active = 0U;
    s_completed = 0U;
    s_state = MOTION_RUNNING;
    s_owns_control = true;
    s_last_error = 0U;
    s_last_update_ms = now_ms;
    s_seg[0].started = 0U;
    return true;
}

void motion_abort(uint32_t now_ms)
{
    s_count = 0U;
    s_expected = 0U;
    s_active = 0U;
    s_completed = 0U;
    s_building = 0U;
    s_state = MOTION_IDLE;
    s_owns_control = false;
    s_cross_track = 0.0f;
    s_heading_error = 0.0f;
    s_cmd_v = 0.0f;
    s_cmd_w = 0.0f;
    (void)give_motion_command(0.0f, 0.0f,
                              CHASSIS_COMMAND_TIMEOUT_DEFAULT_MS, now_ms);
}

void motion_set_limits(float max_speed_mps, float max_omega_radps)
{
    if (motion_finite(max_speed_mps))
    {
        s_max_speed = motion_clamp(max_speed_mps, 0.05f, CHASSIS_MAX_LINEAR_MPS);
    }
    if (motion_finite(max_omega_radps))
    {
        s_max_omega = motion_clamp(max_omega_radps, 0.10f, CHASSIS_MAX_ANGULAR_RADPS);
    }
}

void motion_get_status(motion_status_t *out)
{
    if (out == NULL)
    {
        return;
    }

    out->state = s_state;
    out->active_segment = s_active;
    out->total_segments = s_expected;
    out->completed_segments = s_completed;
    out->building = s_building;
    out->last_error = s_last_error;
    out->target_x_m = s_target_x;
    out->target_y_m = s_target_y;
    out->cross_track_m = s_cross_track;
    out->heading_error_rad = s_heading_error;
    out->command_v_mps = s_cmd_v;
    out->command_w_radps = s_cmd_w;
    out->max_speed_mps = s_max_speed;
    out->max_omega_radps = s_max_omega;
}

void motion_control_update(uint32_t now_ms)
{
    nav_snapshot_t nav;
    motion_seg_t *seg;
    float dx;
    float dy;
    float distance;
    float desired_theta;
    float error;
    float v;
    float w;

    if ((s_state != MOTION_RUNNING) || (!s_owns_control))
    {
        return;
    }
    if ((uint32_t)(now_ms - s_last_update_ms) < 20U)
    {
        return;
    }
    s_last_update_ms = now_ms;

    get_nav_snapshot(&nav);
    if (!nav.ekf_ok)
    {
        motion_fail(6U, now_ms);
        return;
    }

    if (s_active >= s_count)
    {
        motion_complete(now_ms);
        return;
    }

    seg = &s_seg[s_active];
    v = 0.0f;
    w = 0.0f;

    if (seg->type == SEG_LINE)
    {
        dx = seg->x_m - nav.x_m;
        dy = seg->y_m - nav.y_m;
        distance = sqrtf((dx * dx) + (dy * dy));
        s_cross_track = 0.0f;
        s_target_x = seg->x_m;
        s_target_y = seg->y_m;

        if (distance < 0.025f)
        {
            (void)motion_next_segment(now_ms);
            return;
        }

        desired_theta = atan2f(dy, dx);
        error = motion_norm_angle(desired_theta - nav.theta_rad);
        s_heading_error = error;

        if (fabsf(error) > 0.55f)
        {
            v = 0.0f;
            w = 2.2f * error;
        }
        else
        {
            v = seg->speed_mps * motion_clamp((distance / 0.30f), 0.25f, 1.0f);
            v *= cosf(error);
            w = 1.8f * error;
        }
    }
    else if (seg->type == SEG_TURN)
    {
        error = motion_norm_angle(seg->theta_rad - nav.theta_rad);
        s_heading_error = error;
        s_cross_track = 0.0f;
        s_target_x = nav.x_m;
        s_target_y = nav.y_m;

        if (fabsf(error) < 0.02f)
        {
            (void)motion_next_segment(now_ms);
            return;
        }

        v = 0.0f;
        w = 2.5f * error;
        if (fabsf(w) < 0.08f)
        {
            w = (w >= 0.0f) ? 0.08f : -0.08f;
        }
    }
    else if (seg->type == SEG_ARC)
    {
        float sign = (seg->sweep_rad >= 0.0f) ? 1.0f : -1.0f;
        float r;
        float angle;
        float step;
        float tangent;

        if (!seg->started)
        {
            seg->center_x = nav.x_m - (sinf(nav.theta_rad) * seg->radius_m * sign);
            seg->center_y = nav.y_m + (cosf(nav.theta_rad) * seg->radius_m * sign);
            seg->last_angle = atan2f(nav.y_m - seg->center_y,
                                     nav.x_m - seg->center_x);
            seg->accum_angle = 0.0f;
            seg->started = 1U;
        }

        dx = nav.x_m - seg->center_x;
        dy = nav.y_m - seg->center_y;
        r = sqrtf((dx * dx) + (dy * dy));
        angle = atan2f(dy, dx);
        step = motion_norm_angle(angle - seg->last_angle);
        seg->last_angle = angle;
        seg->accum_angle += step;

        if (fabsf(seg->accum_angle) >= fabsf(seg->sweep_rad))
        {
            (void)motion_next_segment(now_ms);
            return;
        }

        tangent = angle + (sign * (NAV_PI_F * 0.5f));
        error = motion_norm_angle(tangent - nav.theta_rad);
        s_heading_error = error;
        s_cross_track = r - seg->radius_m;
        s_target_x = seg->center_x + (cosf(angle) * seg->radius_m);
        s_target_y = seg->center_y + (sinf(angle) * seg->radius_m);

        if (fabsf(error) > 0.60f)
        {
            v = 0.0f;
            w = 2.2f * error;
        }
        else
        {
            v = seg->speed_mps;
            w = (sign * (v / seg->radius_m)) + (1.8f * error);
        }
    }
    else
    {
        motion_fail(7U, now_ms);
        return;
    }

    w = motion_clamp(w, -s_max_omega, s_max_omega);
    motion_send(v, w, now_ms);
    if (s_state == MOTION_ERROR)
    {
        (void)give_motion_command(0.0f, 0.0f,
                                  CHASSIS_COMMAND_TIMEOUT_DEFAULT_MS, now_ms);
    }
}
