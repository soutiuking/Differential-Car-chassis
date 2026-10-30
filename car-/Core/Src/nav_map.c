/**
  ******************************************************************************
  * @file    nav_map.c
  * @brief   已知 4 m × 4 m 围挡的射线-墙线求交。
  ******************************************************************************
  */
#include "nav_map.h"
#include "nav_config.h"
#include "tof400.h"
#include "math.h"
#include <stddef.h>

#define NAV_EDGE_MARGIN_M   0.030f

typedef struct
{
    float offset_x;
    float offset_y;
    float angle;
} nav_tof_mount_t;

static const nav_tof_mount_t s_mount[2] =
{
    { NAV_TOF_FRONT_OFFSET_X_M, NAV_TOF_FRONT_OFFSET_Y_M, NAV_TOF_FRONT_ANGLE_RAD },
    { NAV_TOF_RIGHT_OFFSET_X_M, NAV_TOF_RIGHT_OFFSET_Y_M, NAV_TOF_RIGHT_ANGLE_RAD }
};

bool nav_map_predict_range(float x, float y, float theta,
                           uint8_t sensor, float *range_m)
{
    const nav_tof_mount_t *mount;
    float c;
    float s;
    float sx;
    float sy;
    float ray_angle;
    float dx;
    float dy;
    float best = 1.0e30f;
    float t;
    float hit;

    if ((range_m == NULL) || (sensor > TOF_RIGHT))
    {
        return false;
    }

    mount = &s_mount[sensor];
    c = cosf(theta);
    s = sinf(theta);

    /* 传感器安装点在车体系下的位置转换到世界系。 */
    sx = x + (c * mount->offset_x) - (s * mount->offset_y);
    sy = y + (s * mount->offset_x) + (c * mount->offset_y);
    ray_angle = theta + mount->angle;
    dx = cosf(ray_angle);
    dy = sinf(ray_angle);

    if ((sx < 0.0f) || (sx > NAV_FIELD_LENGTH_M) ||
        (sy < 0.0f) || (sy > NAV_FIELD_WIDTH_M))
    {
        return false;
    }

    /* x = 0 墙 */
    if (dx < -1.0e-6f)
    {
        t = -sx / dx;
        hit = sy + (dy * t);
        if ((t > 0.0f) && (hit > NAV_EDGE_MARGIN_M) &&
            (hit < (NAV_FIELD_WIDTH_M - NAV_EDGE_MARGIN_M)) && (t < best))
        {
            best = t;
        }
    }

    /* x = L 墙 */
    if (dx > 1.0e-6f)
    {
        t = (NAV_FIELD_LENGTH_M - sx) / dx;
        hit = sy + (dy * t);
        if ((t > 0.0f) && (hit > NAV_EDGE_MARGIN_M) &&
            (hit < (NAV_FIELD_WIDTH_M - NAV_EDGE_MARGIN_M)) && (t < best))
        {
            best = t;
        }
    }

    /* y = 0 墙 */
    if (dy < -1.0e-6f)
    {
        t = -sy / dy;
        hit = sx + (dx * t);
        if ((t > 0.0f) && (hit > NAV_EDGE_MARGIN_M) &&
            (hit < (NAV_FIELD_LENGTH_M - NAV_EDGE_MARGIN_M)) && (t < best))
        {
            best = t;
        }
    }

    /* y = L 墙 */
    if (dy > 1.0e-6f)
    {
        t = (NAV_FIELD_WIDTH_M - sy) / dy;
        hit = sx + (dx * t);
        if ((t > 0.0f) && (hit > NAV_EDGE_MARGIN_M) &&
            (hit < (NAV_FIELD_LENGTH_M - NAV_EDGE_MARGIN_M)) && (t < best))
        {
            best = t;
        }
    }

    if (best > 1.0e20f)
    {
        return false;
    }

    *range_m = best;
    return true;
}
