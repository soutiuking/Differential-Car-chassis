/**
  ******************************************************************************
  * @file    nav_ekf.c
  * @brief   6 状态平面 EKF 实现。
  *
  * 预测：轮速 v/omega 推位置，陀螺角速度推航向。
  * 更新：轮速 v、轮速差 omega、陀螺 omega+bias、ToF 墙距。
  ******************************************************************************
  */
#include "nav_ekf.h"
#include "nav_config.h"
#include "nav_map.h"
#include <math.h>
#include <string.h>

#define NAV_STATE_DIM   6U

static float s_state[NAV_STATE_DIM];
static float s_P[NAV_STATE_DIM][NAV_STATE_DIM];

static bool ekf_finite(float v)
{
    return (v == v) && (v <= 1.0e30f) && (v >= -1.0e30f);
}

static float ekf_clamp(float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

static float ekf_normalize_angle(float angle)
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

static void ekf_clamp_pose(void)
{
    s_state[0] = ekf_clamp(s_state[0], -0.2f, NAV_FIELD_LENGTH_M + 0.2f);
    s_state[1] = ekf_clamp(s_state[1], -0.2f, NAV_FIELD_WIDTH_M + 0.2f);
    s_state[2] = ekf_normalize_angle(s_state[2]);
}

static void mat_identity(float m[NAV_STATE_DIM][NAV_STATE_DIM])
{
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < NAV_STATE_DIM; i++)
    {
        for (j = 0U; j < NAV_STATE_DIM; j++)
        {
            m[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

static void mat_mul(float a[NAV_STATE_DIM][NAV_STATE_DIM],
                    float b[NAV_STATE_DIM][NAV_STATE_DIM],
                    float out[NAV_STATE_DIM][NAV_STATE_DIM])
{
    float tmp[NAV_STATE_DIM][NAV_STATE_DIM];
    uint8_t i;
    uint8_t j;
    uint8_t k;

    for (i = 0U; i < NAV_STATE_DIM; i++)
    {
        for (j = 0U; j < NAV_STATE_DIM; j++)
        {
            tmp[i][j] = 0.0f;
            for (k = 0U; k < NAV_STATE_DIM; k++)
            {
                tmp[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    memcpy(out, tmp, sizeof(tmp));
}

static void mat_symmetrize(float m[NAV_STATE_DIM][NAV_STATE_DIM])
{
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < NAV_STATE_DIM; i++)
    {
        for (j = (uint8_t)(i + 1U); j < NAV_STATE_DIM; j++)
        {
            float avg = 0.5f * (m[i][j] + m[j][i]);
            m[i][j] = avg;
            m[j][i] = avg;
        }
    }
}

/* 单维测量更新；返回 false 表示新息超门限。 */
static bool ekf_scalar_update(float z, float h,
                              const float H[NAV_STATE_DIM],
                              float R, float gate_sigma, float gate_base)
{
    float S = R;
    float y;
    float K[NAV_STATE_DIM];
    float HP[NAV_STATE_DIM];
    float gate;
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < NAV_STATE_DIM; i++)
    {
        for (j = 0U; j < NAV_STATE_DIM; j++)
        {
            S += H[i] * s_P[i][j] * H[j];
        }
    }
    if (S <= 1.0e-9f)
    {
        return false;
    }

    y = z - h;
    gate = gate_base + (gate_sigma * sqrtf(S));
    if (fabsf(y) > gate)
    {
        return false;
    }

    for (i = 0U; i < NAV_STATE_DIM; i++)
    {
        float sum = 0.0f;
        for (j = 0U; j < NAV_STATE_DIM; j++)
        {
            sum += s_P[i][j] * H[j];
        }
        K[i] = sum / S;
    }

    for (i = 0U; i < NAV_STATE_DIM; i++)
    {
        s_state[i] += K[i] * y;
    }

    /* P = (I - K H) P */
    for (j = 0U; j < NAV_STATE_DIM; j++)
    {
        float sum = 0.0f;
        for (i = 0U; i < NAV_STATE_DIM; i++)
        {
            sum += H[i] * s_P[i][j];
        }
        HP[j] = sum;
    }
    for (i = 0U; i < NAV_STATE_DIM; i++)
    {
        for (j = 0U; j < NAV_STATE_DIM; j++)
        {
            s_P[i][j] -= K[i] * HP[j];
        }
    }

    s_state[2] = ekf_normalize_angle(s_state[2]);
    mat_symmetrize(s_P);
    for (i = 0U; i < NAV_STATE_DIM; i++)
    {
        if (s_P[i][i] < 1.0e-9f)
        {
            s_P[i][i] = 1.0e-9f;
        }
    }
    return true;
}

static nav_tof_result_t ekf_tof_update(uint8_t sensor, float range_m)
{
    float base;
    float H[NAV_STATE_DIM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float xp;
    float yp;
    float ap;
    float hp;
    float hm;
    float eps_xy = 0.001f;
    float eps_th = 0.001f;
    float S = 0.0f;
    float y;
    float gate;
    float R;
    uint8_t i;

    if (!ekf_finite(range_m) || (range_m < NAV_TOF_MIN_RANGE_M) ||
        (range_m > NAV_TOF_MAX_RANGE_M))
    {
        return NAV_TOF_REJECT;
    }

    if (!nav_map_predict_range(s_state[0], s_state[1], s_state[2], sensor, &base))
    {
        return NAV_TOF_NO_MODEL;
    }

    xp = s_state[0] + eps_xy;
    if (!nav_map_predict_range(xp, s_state[1], s_state[2], sensor, &hp))
    {
        return NAV_TOF_NO_MODEL;
    }
    xp = s_state[0] - eps_xy;
    if (!nav_map_predict_range(xp, s_state[1], s_state[2], sensor, &hm))
    {
        return NAV_TOF_NO_MODEL;
    }
    H[0] = (hp - hm) / (2.0f * eps_xy);

    yp = s_state[1] + eps_xy;
    if (!nav_map_predict_range(s_state[0], yp, s_state[2], sensor, &hp))
    {
        return NAV_TOF_NO_MODEL;
    }
    yp = s_state[1] - eps_xy;
    if (!nav_map_predict_range(s_state[0], yp, s_state[2], sensor, &hm))
    {
        return NAV_TOF_NO_MODEL;
    }
    H[1] = (hp - hm) / (2.0f * eps_xy);

    ap = s_state[2] + eps_th;
    if (!nav_map_predict_range(s_state[0], s_state[1], ap, sensor, &hp))
    {
        return NAV_TOF_NO_MODEL;
    }
    ap = s_state[2] - eps_th;
    if (!nav_map_predict_range(s_state[0], s_state[1], ap, sensor, &hm))
    {
        return NAV_TOF_NO_MODEL;
    }
    H[2] = (hp - hm) / (2.0f * eps_th);

    /* 完整 H P H^T，包含非对角项。 */
    {
        uint8_t j;
        for (i = 0U; i < NAV_STATE_DIM; i++)
        {
            for (j = 0U; j < NAV_STATE_DIM; j++)
            {
                S += H[i] * s_P[i][j] * H[j];
            }
        }
    }
    S += (NAV_EKF_R_TOF_BASE * NAV_EKF_R_TOF_BASE) +
         (NAV_EKF_R_TOF_SCALE * NAV_EKF_R_TOF_SCALE * base * base);
    if (S <= 1.0e-9f)
    {
        return NAV_TOF_REJECT;
    }

    y = range_m - base;
    gate = NAV_TOF_INNOVATION_BASE_M + (NAV_TOF_GATE_SIGMA * sqrtf(S));

    if (y < -(gate + NAV_TOF_OBSTACLE_MARGIN_M))
    {
        return NAV_TOF_OBSTACLE;
    }
    if (fabsf(y) > gate)
    {
        return NAV_TOF_REJECT;
    }

    R = (NAV_EKF_R_TOF_BASE * NAV_EKF_R_TOF_BASE) +
        (NAV_EKF_R_TOF_SCALE * NAV_EKF_R_TOF_SCALE * base * base);
    if (!ekf_scalar_update(range_m, base, H, R,
                           NAV_TOF_GATE_SIGMA, NAV_TOF_INNOVATION_BASE_M))
    {
        return NAV_TOF_REJECT;
    }
    ekf_clamp_pose();
    return NAV_TOF_ACCEPT;
}

void nav_ekf_init(uint32_t now_ms)
{
    (void)now_ms;

    memset(s_state, 0, sizeof(s_state));
    memset(s_P, 0, sizeof(s_P));

    s_state[0] = NAV_START_X_M;
    s_state[1] = NAV_START_Y_M;
    s_state[2] = NAV_START_THETA_RAD;
    s_state[3] = 0.0f;
    s_state[4] = 0.0f;
    s_state[5] = 0.0f;

    s_P[0][0] = 0.30f * 0.30f;
    s_P[1][1] = 0.30f * 0.30f;
    s_P[2][2] = 0.15f * 0.15f;
    s_P[3][3] = 0.10f * 0.10f;
    s_P[4][4] = 0.20f * 0.20f;
    s_P[5][5] = 0.01f * 0.01f;
}

void nav_ekf_set_pose(float x_m, float y_m, float theta_rad)
{
    if (!ekf_finite(x_m) || !ekf_finite(y_m) || !ekf_finite(theta_rad))
    {
        return;
    }

    s_state[0] = x_m;
    s_state[1] = y_m;
    s_state[2] = ekf_normalize_angle(theta_rad);
    s_P[0][0] = 0.05f * 0.05f;
    s_P[1][1] = 0.05f * 0.05f;
    s_P[2][2] = 0.05f * 0.05f;
}

void nav_ekf_set_gyro_bias(float bias_radps)
{
    if (!ekf_finite(bias_radps))
    {
        return;
    }
    s_state[5] = bias_radps;
    s_P[5][5] = 0.005f * 0.005f;
}

void nav_ekf_predict(float dt_s)
{
    float F[NAV_STATE_DIM][NAV_STATE_DIM];
    float FP[NAV_STATE_DIM][NAV_STATE_DIM];
    float Ft[NAV_STATE_DIM][NAV_STATE_DIM];
    float FPFt[NAV_STATE_DIM][NAV_STATE_DIM];
    float scale;
    float c;
    float s;
    uint8_t i;
    uint8_t j;

    if (!ekf_finite(dt_s) || (dt_s <= 0.0f) || (dt_s > 0.20f))
    {
        return;
    }

    c = cosf(s_state[2]);
    s = sinf(s_state[2]);

    s_state[0] += s_state[3] * c * dt_s;
    s_state[1] += s_state[3] * s * dt_s;
    s_state[2] = ekf_normalize_angle(s_state[2] + (s_state[4] * dt_s));
    ekf_clamp_pose();

    mat_identity(F);
    F[0][2] = -s_state[3] * s * dt_s;
    F[0][3] = c * dt_s;
    F[1][2] = s_state[3] * c * dt_s;
    F[1][3] = s * dt_s;
    F[2][4] = dt_s;

    mat_mul(F, s_P, FP);
    for (i = 0U; i < NAV_STATE_DIM; i++)
    {
        for (j = 0U; j < NAV_STATE_DIM; j++)
        {
            Ft[i][j] = F[j][i];
        }
    }
    mat_mul(FP, Ft, FPFt);
    memcpy(s_P, FPFt, sizeof(s_P));

    scale = ekf_clamp(dt_s / (NAV_PERIOD_MS * 0.001f), 0.2f, 5.0f);
    s_P[0][0] += NAV_EKF_Q_POS * scale;
    s_P[1][1] += NAV_EKF_Q_POS * scale;
    s_P[2][2] += NAV_EKF_Q_THETA * scale;
    s_P[3][3] += NAV_EKF_Q_V * scale;
    s_P[4][4] += NAV_EKF_Q_OMEGA * scale;
    s_P[5][5] += NAV_EKF_Q_BIAS * scale;
    mat_symmetrize(s_P);
}

void nav_ekf_update_wheel(float v_mps, float omega_radps)
{
    float H[NAV_STATE_DIM] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};

    if (ekf_finite(v_mps) && (fabsf(v_mps) < 5.0f))
    {
        (void)ekf_scalar_update(v_mps, s_state[3], H,
                                NAV_EKF_R_WHEEL_V * NAV_EKF_R_WHEEL_V,
                                4.0f, 0.0f);
    }

    H[3] = 0.0f;
    H[4] = 1.0f;
    if (ekf_finite(omega_radps) && (fabsf(omega_radps) < 20.0f))
    {
        (void)ekf_scalar_update(omega_radps, s_state[4], H,
                                NAV_EKF_R_WHEEL_OMEGA * NAV_EKF_R_WHEEL_OMEGA,
                                4.0f, 0.0f);
    }
}

void nav_ekf_update_gyro(float gyro_z_radps)
{
    float H[NAV_STATE_DIM] = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f};

    if (!ekf_finite(gyro_z_radps) || (fabsf(gyro_z_radps) > 30.0f))
    {
        return;
    }

    (void)ekf_scalar_update(gyro_z_radps,
                            s_state[4] + s_state[5],
                            H,
                            NAV_EKF_R_GYRO * NAV_EKF_R_GYRO,
                            4.0f, 0.0f);
}

nav_tof_result_t nav_ekf_update_tof(uint8_t sensor, float range_m)
{
    return ekf_tof_update(sensor, range_m);
}

void nav_ekf_get_state(float state[6], float p_diag[6])
{
    uint8_t i;

    if (state != NULL)
    {
        for (i = 0U; i < NAV_STATE_DIM; i++)
        {
            state[i] = s_state[i];
        }
    }
    if (p_diag != NULL)
    {
        for (i = 0U; i < NAV_STATE_DIM; i++)
        {
            p_diag[i] = s_P[i][i];
        }
    }
}
