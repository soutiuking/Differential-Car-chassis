/**
  ******************************************************************************
  * @file    nav_task.c
  * @brief   传感器采集、EKF 更新、ToF 门控和 OLED 显示。
  ******************************************************************************
  */
#include "nav_task.h"
#include "nav_config.h"
#include "nav_ekf.h"
#include "nav_vars.h"
#include "nav_map.h"
#include "mpu6050.h"
#include "tof400.h"
#include "ssd1306.h"
#include "chassis_vars.h"
#include "main.h"
#include <math.h>

#define NAV_DEG_TO_RAD   0.01745329252f
#define NAV_RAD_TO_DEG   57.29577951f
#define NAV_TOF_EXPIRE_MS 800U
#define NAV_DISPLAY_MAX_ERRORS 5U

static bool s_initialized;
static bool s_imu_ok;
static bool s_display_ok;
static uint8_t s_tof_mask;
static uint32_t s_last_nav_ms;
static uint32_t s_last_mpu_ms;
static uint32_t s_mpu_next_try_ms;
static uint32_t s_last_tof_poll_ms;
static uint32_t s_last_display_ms;
static uint8_t s_display_page;
static uint8_t s_display_errors;
static nav_display_mode_t s_display_mode;
static nav_display_mode_t s_last_build_mode;
static bool s_display_force_clear;
static uint8_t s_tof_next;
static float s_roll_rad;
static float s_pitch_rad;

static uint16_t s_tof_mm[2];
static uint8_t s_tof_status[2];
static bool s_tof_valid[2];
static bool s_tof_obstacle[2];
static uint32_t s_tof_updates[2];
static uint32_t s_tof_last_ms[2];

static float nav_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float nav_clamp(float value, float lo, float hi)
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

static void nav_update_attitude(const mpu6050_sample_t *sample, float dt_s)
{
    float ax = sample->accel_g[0];
    float ay = sample->accel_g[1];
    float az = sample->accel_g[2];
    float acc_roll;
    float acc_pitch;
    float gyro_roll;
    float gyro_pitch;
    float alpha = 0.98f;

    acc_roll = atan2f(ay, az);
    acc_pitch = atan2f(-ax, sqrtf((ay * ay) + (az * az)));
    gyro_roll = sample->gyro_dps[0] * NAV_DEG_TO_RAD;
    gyro_pitch = sample->gyro_dps[1] * NAV_DEG_TO_RAD;

    s_roll_rad = (alpha * (s_roll_rad + (gyro_roll * dt_s))) +
                 ((1.0f - alpha) * acc_roll);
    s_pitch_rad = (alpha * (s_pitch_rad + (gyro_pitch * dt_s))) +
                  ((1.0f - alpha) * acc_pitch);
}

static void nav_wheel_velocity(float *v_mps, float *omega_radps)
{
    float left_rpm;
    float right_rpm;
    float rpm_to_mps;
    float v_left;
    float v_right;

    left_rpm = 0.5f * (get_wheel_measured_rpm(CHASSIS_WHEEL_M1) +
                       get_wheel_measured_rpm(CHASSIS_WHEEL_M3));
    right_rpm = 0.5f * (get_wheel_measured_rpm(CHASSIS_WHEEL_M2) +
                        get_wheel_measured_rpm(CHASSIS_WHEEL_M4));

    left_rpm *= NAV_WHEEL_LEFT_SIGN;
    right_rpm *= NAV_WHEEL_RIGHT_SIGN;

    rpm_to_mps = (NAV_PI_F * CHASSIS_WHEEL_DIAMETER_M) / 60.0f;
    v_left = left_rpm * rpm_to_mps;
    v_right = right_rpm * rpm_to_mps;

    *v_mps = 0.5f * (v_left + v_right);
    *omega_radps = (v_right - v_left) / CHASSIS_TRACK_WIDTH_M;
}

static void nav_tof_poll(uint32_t now_ms)
{
    tof_sample_t sample;
    uint8_t sensor;
    nav_tof_result_t result;

    if ((uint32_t)(now_ms - s_last_tof_poll_ms) < NAV_TOF_POLL_PERIOD_MS)
    {
        return;
    }
    s_last_tof_poll_ms = now_ms;

    sensor = s_tof_next;
    s_tof_next ^= 1U;

    if (!tof400_read(sensor, &sample))
    {
        return;
    }

    s_tof_mm[sensor] = sample.distance_mm;
    s_tof_status[sensor] = sample.range_status;

    if (!sample.valid)
    {
        s_tof_valid[sensor] = false;
        s_tof_obstacle[sensor] = false;
        return;
    }

    if (get_nav_mode() == 0U)
    {
        /* 日常测试模式：ToF 只显示/标定，不参与 EKF。 */
        s_tof_valid[sensor] = sample.valid;
        s_tof_obstacle[sensor] = false;
        s_tof_last_ms[sensor] = now_ms;
        if (sample.valid && (s_tof_updates[sensor] != 0xFFFFFFFFU))
        {
            s_tof_updates[sensor]++;
        }
        return;
    }

    result = nav_ekf_update_tof(sensor, (float)sample.distance_mm / 1000.0f);
    if (result == NAV_TOF_ACCEPT)
    {
        s_tof_valid[sensor] = true;
        s_tof_obstacle[sensor] = false;
        s_tof_last_ms[sensor] = now_ms;
        if (s_tof_updates[sensor] != 0xFFFFFFFFU)
        {
            s_tof_updates[sensor]++;
        }
    }
    else if (result == NAV_TOF_OBSTACLE)
    {
        s_tof_valid[sensor] = false;
        s_tof_obstacle[sensor] = true;
        s_tof_last_ms[sensor] = now_ms;
    }
    else
    {
        s_tof_valid[sensor] = false;
        s_tof_obstacle[sensor] = false;
    }
}

static void nav_tof_expire(uint32_t now_ms)
{
    uint8_t i;

    for (i = 0U; i < 2U; i++)
    {
        if ((uint32_t)(now_ms - s_tof_last_ms[i]) > NAV_TOF_EXPIRE_MS)
        {
            s_tof_valid[i] = false;
            s_tof_obstacle[i] = false;
        }
    }
}

static void nav_publish(uint32_t now_ms)
{
    nav_snapshot_t snap;
    float state[6];
    float p_diag[6];

    nav_ekf_get_state(state, p_diag);

    snap.ekf_ok = true;
    snap.imu_ok = s_imu_ok;
    snap.mode = get_nav_mode();
    snap.x_m = state[0];
    snap.y_m = state[1];
    snap.theta_rad = state[2];
    snap.v_mps = state[3];
    snap.omega_radps = state[4];
    snap.gyro_bias_radps = state[5];
    snap.p_xx = p_diag[0];
    snap.p_yy = p_diag[1];
    snap.p_tt = p_diag[2];
    snap.roll_rad = s_roll_rad;
    snap.pitch_rad = s_pitch_rad;

    snap.tof_mm[0] = s_tof_mm[0];
    snap.tof_mm[1] = s_tof_mm[1];
    snap.tof_status[0] = s_tof_status[0];
    snap.tof_status[1] = s_tof_status[1];
    snap.tof_valid[0] = s_tof_valid[0];
    snap.tof_valid[1] = s_tof_valid[1];
    snap.tof_obstacle[0] = s_tof_obstacle[0];
    snap.tof_obstacle[1] = s_tof_obstacle[1];
    snap.tof_updates[0] = s_tof_updates[0];
    snap.tof_updates[1] = s_tof_updates[1];
    snap.last_update_ms = now_ms;

    (void)nav_give_snapshot(&snap);
}

static void nav_display_build_normal(void)
{
    nav_snapshot_t snap;
    int32_t x_cm;
    int32_t y_cm;
    int32_t theta_deg;
    int32_t v_cms;
    const char *mode_text;
    const char *imu_text;
    uint8_t tof_mask;

    get_nav_snapshot(&snap);
    x_cm = (int32_t)(snap.x_m * 100.0f);
    y_cm = (int32_t)(snap.y_m * 100.0f);
    theta_deg = (int32_t)(snap.theta_rad * NAV_RAD_TO_DEG);
    v_cms = (int32_t)(snap.v_mps * 100.0f);
    mode_text = (snap.mode == 1U) ? "MODE COMP" : "MODE DAILY";
    imu_text = snap.imu_ok ? "OK" : "ERR";
    tof_mask = (uint8_t)((snap.tof_valid[0] ? 1U : 0U) |
                         (snap.tof_valid[1] ? 2U : 0U));

    ssd1306_draw_text(0U, 0U, "SPD");
    ssd1306_draw_i32(18U, 0U, v_cms, 4U);
    ssd1306_draw_text(42U, 0U, "CM/S");

    ssd1306_draw_text(0U, 1U, "TH");
    ssd1306_draw_i32(12U, 1U, theta_deg, 4U);
    ssd1306_draw_text(36U, 1U, "DEG");

    ssd1306_draw_text(0U, 2U, "X");
    ssd1306_draw_i32(6U, 2U, x_cm, 4U);
    ssd1306_draw_text(30U, 2U, "CM");

    ssd1306_draw_text(0U, 3U, "Y");
    ssd1306_draw_i32(6U, 3U, y_cm, 4U);
    ssd1306_draw_text(30U, 3U, "CM");

    ssd1306_draw_text(0U, 4U, mode_text);
    ssd1306_draw_text(0U, 5U, "TOF");
    ssd1306_draw_i32(18U, 5U, (int32_t)tof_mask, 1U);
    ssd1306_draw_text(30U, 5U, "IMU");
    ssd1306_draw_text(48U, 5U, imu_text);

    ssd1306_draw_text(0U, 6U, "K1 EN  K2 MODE");
    ssd1306_draw_text(0U, 7U, "K3 CAL K4 DEBUG");
}

static void nav_display_build_debug(void)
{
    nav_snapshot_t snap;
    int32_t x_cm;
    int32_t y_cm;
    int32_t theta_deg;
    int32_t v_cms;
    int32_t pxx_cm2;
    int32_t pyy_cm2;
    const char *state_text;

    get_nav_snapshot(&snap);
    x_cm = (int32_t)(snap.x_m * 100.0f);
    y_cm = (int32_t)(snap.y_m * 100.0f);
    theta_deg = (int32_t)(snap.theta_rad * NAV_RAD_TO_DEG);
    v_cms = (int32_t)(snap.v_mps * 100.0f);
    pxx_cm2 = (int32_t)(snap.p_xx * 10000.0f);
    pyy_cm2 = (int32_t)(snap.p_yy * 10000.0f);
    if (pxx_cm2 > 9999) pxx_cm2 = 9999;
    if (pyy_cm2 > 9999) pyy_cm2 = 9999;

    ssd1306_draw_text(0U, 0U, "X");
    ssd1306_draw_i32(12U, 0U, x_cm, 4U);
    ssd1306_draw_text(36U, 0U, "CM Y");
    ssd1306_draw_i32(66U, 0U, y_cm, 4U);

    ssd1306_draw_text(0U, 1U, "TH");
    ssd1306_draw_i32(12U, 1U, theta_deg, 4U);
    ssd1306_draw_text(36U, 1U, " V");
    ssd1306_draw_i32(48U, 1U, v_cms, 4U);

    ssd1306_draw_text(0U, 2U, "TOF F");
    ssd1306_draw_i32(36U, 2U, (int32_t)snap.tof_mm[0], 4U);
    ssd1306_draw_text(60U, 2U, " R");
    ssd1306_draw_i32(72U, 2U, (int32_t)snap.tof_mm[1], 4U);

    state_text = snap.imu_ok ? "OK" : "ERR";
    ssd1306_draw_text(0U, 3U, "IMU");
    ssd1306_draw_text(24U, 3U, state_text);
    ssd1306_draw_text(48U, 3U, "TOF");
    ssd1306_draw_i32(72U, 3U,
                     (int32_t)((snap.tof_valid[0] ? 1 : 0) +
                               (snap.tof_valid[1] ? 2 : 0)), 1U);

    ssd1306_draw_text(0U, 4U, "PXX");
    ssd1306_draw_i32(18U, 4U, pxx_cm2, 4U);
    ssd1306_draw_text(42U, 4U, "YY");
    ssd1306_draw_i32(54U, 4U, pyy_cm2, 4U);

    ssd1306_draw_text(0U, 5U, "M1");
    ssd1306_draw_i32(12U, 5U,
                     (int32_t)get_wheel_measured_rpm(CHASSIS_WHEEL_M1), 5U);
    ssd1306_draw_text(42U, 5U, "M2");
    ssd1306_draw_i32(54U, 5U,
                     (int32_t)get_wheel_measured_rpm(CHASSIS_WHEEL_M2), 5U);

    ssd1306_draw_text(0U, 6U, "M3");
    ssd1306_draw_i32(12U, 6U,
                     (int32_t)get_wheel_measured_rpm(CHASSIS_WHEEL_M3), 5U);
    ssd1306_draw_text(42U, 6U, "M4");
    ssd1306_draw_i32(54U, 6U,
                     (int32_t)get_wheel_measured_rpm(CHASSIS_WHEEL_M4), 5U);

    ssd1306_draw_text(0U, 7U, "EKF");
    ssd1306_draw_text(18U, 7U, snap.ekf_ok ? "OK" : "ERR");
    ssd1306_draw_text(42U, 7U, "UPD");
    ssd1306_draw_i32(60U, 7U,
                     (int32_t)(snap.tof_updates[0] + snap.tof_updates[1]), 5U);
}

static void nav_display_build_calib(void)
{
    ssd1306_draw_text(0U, 0U, "MPU6050");
    ssd1306_draw_text(0U, 1U, "CALIBRATING");
    ssd1306_draw_text(0U, 2U, "KEEP STILL");
    ssd1306_draw_text(0U, 3U, "CAR DISABLED");
}

static void nav_display_build(void)
{
    uint8_t page;

    if (!s_display_ok)
    {
        return;
    }

    if (s_display_force_clear || (s_display_mode != s_last_build_mode))
    {
        ssd1306_clear();
        s_last_build_mode = s_display_mode;
        s_display_force_clear = false;
    }

    /* 每页先清空，避免数字变短时残留旧像素；整屏只在状态切换时清。 */
    for (page = 0U; page < 8U; page++)
    {
        ssd1306_clear_page(page);
    }

    if (s_display_mode == NAV_DISPLAY_NORMAL)
    {
        nav_display_build_normal();
    }
    else if (s_display_mode == NAV_DISPLAY_DEBUG)
    {
        nav_display_build_debug();
    }
    else
    {
        nav_display_build_calib();
    }
}

static void nav_display_poll(uint32_t now_ms)
{
    if ((!s_display_ok) ||
        ((uint32_t)(now_ms - s_last_display_ms) < NAV_DISPLAY_PERIOD_MS))
    {
        return;
    }
    s_last_display_ms = now_ms;

    if (s_display_page == 0U)
    {
        nav_display_build();
    }

    if (ssd1306_flush_page(s_display_page))
    {
        s_display_errors = 0U;
        s_display_page = (uint8_t)((s_display_page + 1U) & 0x07U);
    }
    else
    {
        if (s_display_errors < 255U)
        {
            s_display_errors++;
        }
        if (s_display_errors >= NAV_DISPLAY_MAX_ERRORS)
        {
            s_display_ok = false;
        }
    }
}

void nav_display_set_mode(nav_display_mode_t mode)
{
    if (mode > NAV_DISPLAY_CALIB)
    {
        return;
    }
    if (mode != s_display_mode)
    {
        s_display_mode = mode;
        s_display_page = 0U;
        s_display_force_clear = true;
        s_last_display_ms = HAL_GetTick();
    }
}

nav_display_mode_t nav_display_get_mode(void)
{
    return s_display_mode;
}

void nav_display_refresh_now(void)
{
    uint8_t page;

    if (!s_display_ok)
    {
        return;
    }

    nav_display_build();
    for (page = 0U; page < 8U; page++)
    {
        (void)ssd1306_flush_page(page);
    }
    s_display_page = 0U;
}

static bool nav_initial_pose_from_tof(void)
{
    tof_sample_t sample;
    bool have_front = false;
    bool have_right = false;
    uint16_t front_mm = 0U;
    uint16_t right_mm = 0U;
    uint8_t i;
    uint8_t attempt;
    float x_m;
    float y_m;

    for (attempt = 0U; (attempt < 60U) && !(have_front && have_right); attempt++)
    {
        for (i = 0U; i < 2U; i++)
        {
            if (tof400_read(i, &sample) && sample.valid)
            {
                if (i == TOF_FRONT)
                {
                    front_mm = sample.distance_mm;
                    have_front = true;
                }
                else
                {
                    right_mm = sample.distance_mm;
                    have_right = true;
                }
            }
        }
        if (!(have_front && have_right))
        {
            HAL_Delay(20U);
        }
    }

    if (!(have_front && have_right))
    {
        return false;
    }

    /* 车头朝 +x，前向到 x=L 墙，右向到 y=0 墙。 */
    x_m = NAV_FIELD_LENGTH_M - NAV_TOF_FRONT_OFFSET_X_M -
          ((float)front_mm / 1000.0f);
    y_m = ((float)right_mm / 1000.0f) - NAV_TOF_RIGHT_OFFSET_Y_M;

    if ((x_m < 0.0f) || (x_m > NAV_FIELD_LENGTH_M) ||
        (y_m < 0.0f) || (y_m > NAV_FIELD_WIDTH_M))
    {
        return false;
    }

    nav_ekf_set_pose(x_m, y_m, NAV_START_THETA_RAD);
    return true;
}

void nav_task_init(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t i;

    nav_vars_init();
    nav_ekf_init(now);

    s_imu_ok = mpu6050_init();
    if (s_imu_ok)
    {
        if (!mpu6050_calibrate_gyro(100U))
        {
            s_imu_ok = false;
        }
    }

    s_tof_mask = tof400_init();
    s_display_ok = ssd1306_init();

    for (i = 0U; i < 2U; i++)
    {
        s_tof_mm[i] = 0U;
        s_tof_status[i] = 0xFFU;
        s_tof_valid[i] = false;
        s_tof_obstacle[i] = false;
        s_tof_updates[i] = 0U;
        s_tof_last_ms[i] = now;
    }

    s_roll_rad = 0.0f;
    s_pitch_rad = 0.0f;
    s_last_nav_ms = now;
    s_last_mpu_ms = now;
    s_mpu_next_try_ms = now;
    s_last_tof_poll_ms = now;
    s_last_display_ms = now;
    s_display_page = 0U;
    s_display_errors = 0U;
    s_display_mode = NAV_DISPLAY_NORMAL;
    s_last_build_mode = NAV_DISPLAY_NORMAL;
    s_display_force_clear = true;
    s_tof_next = 0U;
    s_initialized = true;
}

void nav_task_initial_pose(void)
{
    if ((s_tof_mask & 0x03U) == 0x03U)
    {
        (void)nav_initial_pose_from_tof();
    }
}

void nav_task_update(uint32_t now_ms)
{
    mpu6050_sample_t sample;
    float v_mps;
    float omega_radps;
    float dt_s;
    bool have_gyro = false;

    if (!s_initialized)
    {
        return;
    }

    if ((uint32_t)(now_ms - s_last_nav_ms) >= NAV_PERIOD_MS)
    {
        dt_s = (float)(now_ms - s_last_nav_ms) * 0.001f;
        dt_s = nav_clamp(dt_s, 0.005f, 0.100f);
        s_last_nav_ms = now_ms;

        if (((uint32_t)(now_ms - s_last_mpu_ms) >= NAV_MPU_PERIOD_MS) &&
            ((int32_t)(now_ms - s_mpu_next_try_ms) >= 0))
        {
            s_last_mpu_ms = now_ms;
            if (mpu6050_read(&sample))
            {
                s_imu_ok = true;
                have_gyro = true;
                nav_update_attitude(&sample, dt_s);
            }
            else
            {
                s_imu_ok = false;
                s_mpu_next_try_ms = now_ms + 1000U;
            }
        }

        nav_wheel_velocity(&v_mps, &omega_radps);

        nav_ekf_predict(dt_s);
        nav_ekf_update_wheel(v_mps, omega_radps);

        if (have_gyro)
        {
            nav_ekf_update_gyro(sample.gyro_dps[2] * NAV_DEG_TO_RAD *
                                NAV_GYRO_Z_SIGN);
        }

        /* 静止时给一个零速约束，抑制轮速噪声引起的慢漂。 */
        if ((!get_chassis_enabled()) &&
            (nav_absf(v_mps) < 0.03f) && (nav_absf(omega_radps) < 0.10f))
        {
            nav_ekf_update_wheel(0.0f, 0.0f);
        }

        nav_tof_poll(now_ms);
        nav_tof_expire(now_ms);
        nav_publish(now_ms);
    }

    nav_display_poll(now_ms);
}
