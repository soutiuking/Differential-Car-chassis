/**
  ******************************************************************************
  * @file    nav_vars.c
  * @brief   导航输出变量存储与校验。
  ******************************************************************************
  */
#include "nav_vars.h"
#include "nav_config.h"
#include "main.h"

static nav_snapshot_t s_nav;

static uint32_t nav_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void nav_exit_critical(uint32_t primask)
{
    if ((primask & 1U) == 0U)
    {
        __enable_irq();
    }
}

static bool nav_finite(float value)
{
    return (value >= -1.0e30f) && (value <= 1.0e30f);
}

void nav_vars_init(void)
{
    uint32_t primask = nav_enter_critical();
    uint8_t i;

    s_nav.ekf_ok = false;
    s_nav.imu_ok = false;
    s_nav.mode = 0U;
    s_nav.x_m = 0.0f;
    s_nav.y_m = 0.0f;
    s_nav.theta_rad = 0.0f;
    s_nav.v_mps = 0.0f;
    s_nav.omega_radps = 0.0f;
    s_nav.gyro_bias_radps = 0.0f;
    s_nav.p_xx = 1.0f;
    s_nav.p_yy = 1.0f;
    s_nav.p_tt = 0.1f;
    s_nav.roll_rad = 0.0f;
    s_nav.pitch_rad = 0.0f;
    for (i = 0U; i < 2U; i++)
    {
        s_nav.tof_mm[i] = 0U;
        s_nav.tof_status[i] = 0xFFU;
        s_nav.tof_valid[i] = false;
        s_nav.tof_obstacle[i] = false;
        s_nav.tof_updates[i] = 0U;
    }
    s_nav.last_update_ms = 0U;

    nav_exit_critical(primask);
}

void get_nav_snapshot(nav_snapshot_t *out)
{
    uint32_t primask;

    if (out == NULL)
    {
        return;
    }

    primask = nav_enter_critical();
    *out = s_nav;
    nav_exit_critical(primask);
}

var_status_t nav_give_snapshot(const nav_snapshot_t *value)
{
    uint32_t primask;
    var_status_t status = VAR_OK;
    uint8_t i;

    if (value == NULL)
    {
        return VAR_ERR_ARG;
    }

    primask = nav_enter_critical();

    if (!nav_finite(value->x_m) || !nav_finite(value->y_m) ||
        !nav_finite(value->theta_rad) || !nav_finite(value->v_mps) ||
        !nav_finite(value->omega_radps) || !nav_finite(value->gyro_bias_radps) ||
        !nav_finite(value->p_xx) || !nav_finite(value->p_yy) ||
        !nav_finite(value->p_tt) || !nav_finite(value->roll_rad) ||
        !nav_finite(value->pitch_rad))
    {
        status = VAR_ERR_RANGE;
    }
    else if ((value->x_m < -1.0f) || (value->x_m > (NAV_FIELD_LENGTH_M + 1.0f)) ||
             (value->y_m < -1.0f) || (value->y_m > (NAV_FIELD_WIDTH_M + 1.0f)) ||
             (value->theta_rad < -NAV_PI_F) || (value->theta_rad > NAV_PI_F) ||
             (value->v_mps < -5.0f) || (value->v_mps > 5.0f) ||
             (value->omega_radps < -20.0f) || (value->omega_radps > 20.0f) ||
             (value->gyro_bias_radps < -5.0f) || (value->gyro_bias_radps > 5.0f) ||
             (value->p_xx < 0.0f) || (value->p_yy < 0.0f) || (value->p_tt < 0.0f) ||
             (value->p_xx > 100.0f) || (value->p_yy > 100.0f) || (value->p_tt > 100.0f) ||
             (value->roll_rad < -NAV_PI_F) || (value->roll_rad > NAV_PI_F) ||
             (value->pitch_rad < -NAV_PI_F) || (value->pitch_rad > NAV_PI_F))
    {
        status = VAR_ERR_RANGE;
    }
    else
    {
        for (i = 0U; i < 2U; i++)
        {
            if (value->tof_mm[i] > 5000U)
            {
                status = VAR_ERR_RANGE;
                break;
            }
        }
    }

    if (status == VAR_OK)
    {
        s_nav = *value;
    }

    nav_exit_critical(primask);
    return status;
}

uint8_t get_nav_mode(void)
{
    uint32_t primask = nav_enter_critical();
    uint8_t mode = s_nav.mode;
    nav_exit_critical(primask);
    return mode;
}

var_status_t give_nav_mode(uint8_t mode)
{
    uint32_t primask;
    var_status_t status = VAR_OK;

    if (mode > 1U)
    {
        return VAR_ERR_RANGE;
    }

    primask = nav_enter_critical();
    s_nav.mode = mode;
    nav_exit_critical(primask);
    return status;
}
