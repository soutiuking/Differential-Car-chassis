#include "car_state.h"
#include <string.h>

car_state_t g_state;
SemaphoreHandle_t g_state_mutex;

static uint16_t rd16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t rds16(const uint8_t *p)
{
    return (int16_t)rd16(p);
}

static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

void state_init(void)
{
    memset(&g_state, 0, sizeof(g_state));
    g_state_mutex = xSemaphoreCreateMutex();
}

void state_lock(void)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
}

void state_unlock(void)
{
    xSemaphoreGive(g_state_mutex);
}

static void state_push_trail(float x, float y)
{
    if (g_state.trail_count == 0U) {
        g_state.trail_x[0] = x;
        g_state.trail_y[0] = y;
        g_state.trail_count = 1U;
        return;
    }
    uint16_t last = (uint16_t)(g_state.trail_count - 1U);
    float dx = x - g_state.trail_x[last];
    float dy = y - g_state.trail_y[last];
    if ((dx * dx + dy * dy) < (0.015f * 0.015f)) {
        return;
    }
    if (g_state.trail_count < TRAIL_MAX_POINTS) {
        g_state.trail_x[g_state.trail_count] = x;
        g_state.trail_y[g_state.trail_count] = y;
        g_state.trail_count++;
    } else {
        memmove(&g_state.trail_x[0], &g_state.trail_x[1],
                (TRAIL_MAX_POINTS - 1U) * sizeof(float));
        memmove(&g_state.trail_y[0], &g_state.trail_y[1],
                (TRAIL_MAX_POINTS - 1U) * sizeof(float));
        g_state.trail_x[TRAIL_MAX_POINTS - 1U] = x;
        g_state.trail_y[TRAIL_MAX_POINTS - 1U] = y;
    }
}

void state_update_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    state_lock();

    if (type == 0x81U && len >= 44U) {
        g_state.have_chassis = true;
        g_state.faults = rd16(&payload[0]);
        g_state.flags = rd16(&payload[2]);
        g_state.linear_mm_s = rds16(&payload[4]);
        g_state.angular_mrad_s = rds16(&payload[6]);
        for (uint8_t i = 0; i < 4U; i++) {
            g_state.target_rpm[i] = rds16(&payload[10U + i * 2U]);
            g_state.measured_rpm[i] = rds16(&payload[18U + i * 2U]);
            g_state.pwm[i] = rds16(&payload[26U + i * 2U]);
        }
    } else if (type == 0x82U && len >= 30U) {
        uint16_t flags = rd16(&payload[0]);
        g_state.have_nav = true;
        g_state.x_m = (float)rds16(&payload[2]) / 1000.0f;
        g_state.y_m = (float)rds16(&payload[4]) / 1000.0f;
        g_state.theta_rad = (float)rds16(&payload[6]) / 1000.0f;
        g_state.v_mps = (float)rds16(&payload[8]) / 1000.0f;
        g_state.omega_radps = (float)rds16(&payload[10]) / 1000.0f;
        g_state.roll_rad = (float)rds16(&payload[12]) / 1000.0f;
        g_state.pitch_rad = (float)rds16(&payload[14]) / 1000.0f;
        g_state.tof_mm[0] = rd16(&payload[16]);
        g_state.tof_mm[1] = rd16(&payload[18]);
        g_state.p_xx = (float)rd16(&payload[20]) / 10000.0f;
        g_state.p_yy = (float)rd16(&payload[22]) / 10000.0f;
        g_state.p_tt = (float)rd16(&payload[24]) / 1000000.0f;
        g_state.tof_valid[0] = (flags & (1U << 2)) != 0U;
        g_state.tof_valid[1] = (flags & (1U << 3)) != 0U;
        g_state.tof_obstacle[0] = (flags & (1U << 4)) != 0U;
        g_state.tof_obstacle[1] = (flags & (1U << 5)) != 0U;
        g_state.nav_mode = (flags & (1U << 6)) ? 1U : 0U;
        state_push_trail(g_state.x_m, g_state.y_m);
    } else if (type == 0x83U && len >= 28U) {
        g_state.have_motion = true;
        g_state.motion_state = payload[0];
        g_state.motion_active = payload[1];
        g_state.motion_total = payload[2];
        g_state.motion_completed = payload[3];
        g_state.motion_building = payload[4];
        g_state.motion_error = payload[5];
        g_state.motion_target_x = (float)rds16(&payload[8]) / 1000.0f;
        g_state.motion_target_y = (float)rds16(&payload[10]) / 1000.0f;
        g_state.motion_cross_track = (float)rds16(&payload[12]) / 1000.0f;
        g_state.motion_heading_error = (float)rds16(&payload[14]) / 1000.0f;
        g_state.motion_cmd_v = (float)rds16(&payload[16]) / 1000.0f;
        g_state.motion_cmd_w = (float)rds16(&payload[18]) / 1000.0f;
        g_state.motion_max_v = (float)rd16(&payload[20]) / 1000.0f;
        g_state.motion_max_w = (float)rd16(&payload[22]) / 1000.0f;
    } else if (type == 0x84U && len >= 8U) {
        g_state.have_param = true;
        g_state.param_id = payload[0];
        g_state.param_status = payload[1];
        g_state.param_value = (int32_t)rd32(&payload[2]);
        if (g_state.param_id < 7U) {
            g_state.params[g_state.param_id] = g_state.param_value;
            g_state.params_valid[g_state.param_id] = true;
        }
    } else if (type == 0x85U && len >= 16U) {
        g_state.have_calib = true;
        g_state.calib_action = payload[0];
        g_state.calib_sensor = payload[1];
        g_state.calib_count = payload[2];
        g_state.calib_status = payload[3];
        g_state.calib_known_mm = rd16(&payload[4]);
        g_state.calib_raw_mm = rd16(&payload[6]);
        g_state.calib_gain = (float)rds16(&payload[8]) / 1000.0f;
        g_state.calib_offset = (float)rds16(&payload[10]);
        if (g_state.calib_sensor < 2U) {
            uint8_t gid = (uint8_t)(3U + g_state.calib_sensor * 2U);
            uint8_t oid = (uint8_t)(gid + 1U);
            g_state.params[gid] = (int32_t)(g_state.calib_gain * 1000.0f);
            g_state.params[oid] = (int32_t)g_state.calib_offset;
            g_state.params_valid[gid] = true;
            g_state.params_valid[oid] = true;
        }
    }

    state_unlock();
}

void state_set_plan(const float *x, const float *y, uint16_t count)
{
    state_lock();
    if (count > PLAN_MAX_POINTS) {
        count = PLAN_MAX_POINTS;
    }
    g_state.plan_count = count;
    for (uint16_t i = 0; i < count; i++) {
        g_state.plan_x[i] = x[i];
        g_state.plan_y[i] = y[i];
    }
    state_unlock();
}

void state_clear_plan(void)
{
    state_lock();
    g_state.plan_count = 0U;
    g_state.trail_count = 0U;
    state_unlock();
}
