#include "path_planner.h"
#include "car_state.h"
#include "uart_link.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

#define PLAN_MAX_SEGMENTS  (PLAN_MAX_POINTS - 1U)
#define PLAN_MAX_SPEED     0.80f

static const char *TAG = "planner";
static float s_x[PLAN_MAX_POINTS];
static float s_y[PLAN_MAX_POINTS];
static uint16_t s_count;
static float s_speed;
static uint8_t s_monitor_bad;

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float dist2(float ax, float ay, float bx, float by)
{
    float dx = ax - bx;
    float dy = ay - by;
    return (dx * dx) + (dy * dy);
}

static void planner_begin(void)
{
    state_lock();
    if (g_state.have_nav) {
        s_x[0] = g_state.x_m;
        s_y[0] = g_state.y_m;
    } else {
        s_x[0] = 0.5f;
        s_y[0] = 0.5f;
    }
    state_unlock();
    s_count = 1U;
    s_monitor_bad = 0U;
}

static bool planner_add(float x, float y)
{
    if (s_count >= PLAN_MAX_POINTS) return false;
    if (x < 0.0f || x > 4.0f || y < 0.0f || y > 4.0f) return false;
    if (dist2(x, y, s_x[s_count - 1U], s_y[s_count - 1U]) < (0.01f * 0.01f)) {
        return true;
    }
    s_x[s_count] = x;
    s_y[s_count] = y;
    s_count++;
    return true;
}

bool planner_send(void)
{
    if (s_count < 2U || (s_count - 1U) > PLAN_MAX_SEGMENTS) {
        return false;
    }

    uint8_t total = (uint8_t)(s_count - 1U);
    uint8_t begin[2] = { total, 0U };
    if (!uart_link_send(0x16U, begin, 2U)) return false;

    for (uint16_t i = 1U; i < s_count; i++) {
        int32_t x_mm = (int32_t)(s_x[i] * 1000.0f);
        int32_t y_mm = (int32_t)(s_y[i] * 1000.0f);
        uint16_t speed = (uint16_t)(clampf(s_speed, 0.03f, PLAN_MAX_SPEED) * 1000.0f);
        uint8_t p[8];
        p[0] = (uint8_t)(x_mm & 0xFF);
        p[1] = (uint8_t)((x_mm >> 8) & 0xFF);
        p[2] = (uint8_t)(y_mm & 0xFF);
        p[3] = (uint8_t)((y_mm >> 8) & 0xFF);
        p[4] = (uint8_t)(speed & 0xFF);
        p[5] = (uint8_t)((speed >> 8) & 0xFF);
        p[6] = 0U;
        p[7] = 0U;
        if (!uart_link_send(0x11U, p, 8U)) {
            (void)uart_link_send(0x18U, NULL, 0U);
            return false;
        }
    }

    if (!uart_link_send(0x17U, NULL, 0U)) return false;
    state_set_plan(s_x, s_y, s_count);
    ESP_LOGI(TAG, "plan sent: %u lines, speed %.2f", total, s_speed);
    return true;
}

void planner_init(void)
{
    s_count = 0U;
    s_speed = 0.25f;
    s_monitor_bad = 0U;
    planner_clear();
}

void planner_clear(void)
{
    s_count = 0U;
    state_clear_plan();
}

bool planner_goto(float x, float y, float speed_mps)
{
    planner_begin();
    s_speed = speed_mps;
    if (!planner_add(x, y)) return false;
    return planner_send();
}

bool planner_line(const float *x, const float *y, uint16_t count, float speed_mps)
{
    if (!x || !y || count == 0U || count > (PLAN_MAX_POINTS - 1U)) return false;
    planner_begin();
    s_speed = speed_mps;
    for (uint16_t i = 0U; i < count; i++) {
        if (!planner_add(x[i], y[i])) return false;
    }
    return planner_send();
}

bool planner_rect(float x1, float y1, float x2, float y2, float speed_mps)
{
    planner_begin();
    s_speed = speed_mps;
    if (!planner_add(x1, y1) || !planner_add(x2, y1) ||
        !planner_add(x2, y2) || !planner_add(x1, y2) ||
        !planner_add(x1, y1)) {
        return false;
    }
    return planner_send();
}

bool planner_circle(float cx, float cy, float radius, bool ccw, float speed_mps)
{
    float start_angle;
    float step = (ccw ? 1.0f : -1.0f) * (2.0f * (float)M_PI / 36.0f);

    if (radius < 0.08f || radius > 2.0f) return false;
    if (cx < 0.0f || cx > 4.0f || cy < 0.0f || cy > 4.0f) return false;

    planner_begin();
    s_speed = speed_mps;

    state_lock();
    start_angle = atan2f(g_state.y_m - cy, g_state.x_m - cx);
    state_unlock();

    for (uint16_t i = 0U; i <= 36U; i++) {
        float a = start_angle + step * (float)i;
        float x = cx + cosf(a) * radius;
        float y = cy + sinf(a) * radius;
        if (!planner_add(x, y)) return false;
    }
    return planner_send();
}

bool planner_polyline(const float *x, const float *y, uint16_t count, float speed_mps)
{
    return planner_line(x, y, count, speed_mps);
}

void planner_monitor_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(200));

        state_lock();
        uint8_t running = (g_state.have_motion && g_state.motion_state == 1U) ? 1U : 0U;
        uint8_t active = g_state.motion_active;
        uint16_t plan_count = g_state.plan_count;
        float px = g_state.x_m;
        float py = g_state.y_m;
        float x0 = 0.0f, y0 = 0.0f, x1 = 0.0f, y1 = 0.0f;
        float speed = 0.0f;
        if (plan_count >= 2U && active < (plan_count - 1U)) {
            x0 = g_state.plan_x[active];
            y0 = g_state.plan_y[active];
            x1 = g_state.plan_x[active + 1U];
            y1 = g_state.plan_y[active + 1U];
            speed = g_state.motion_cmd_v;
        }
        state_unlock();

        if (!running || plan_count < 2U) {
            s_monitor_bad = 0U;
            continue;
        }

        float dx = x1 - x0;
        float dy = y1 - y0;
        float len2 = dx * dx + dy * dy;
        float t = 0.0f;
        if (len2 > 1.0e-6f) {
            t = ((px - x0) * dx + (py - y0) * dy) / len2;
            t = clampf(t, 0.0f, 1.0f);
        }
        float ex = px - (x0 + t * dx);
        float ey = py - (y0 + t * dy);
        float err = sqrtf(ex * ex + ey * ey);

        if (err > 0.50f) {
            s_monitor_bad++;
            if (s_monitor_bad >= 3U) {
                ESP_LOGW(TAG, "path deviation %.2f m, abort (v=%.2f)", err, speed);
                (void)uart_link_send(0x18U, NULL, 0U);
                state_clear_plan();
                s_monitor_bad = 0U;
            }
        } else {
            s_monitor_bad = 0U;
        }
    }
}
