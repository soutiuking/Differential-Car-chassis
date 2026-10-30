/**
  ******************************************************************************
  * @file    chassis_control.c
  * @brief   四轮差速底盘速度环。
  *
  * 控制流程（每 10 ms）：
  *   读编码器 -> 轮端 rpm -> 差速运动学分解 -> 四轮 PID -> TB6612 输出
  *
  * 安全策略：
  *   - 命令超时自动断使能并置故障；
  *   - 堵转、测速异常、目标超限都会立即退出使能；
  *   - 任何停止路径都先滑行，再清 PID 和目标；
  *   - 换向有 25 ms 滑行死区，PWM 有斜率限制。
  ******************************************************************************
  */
#include "chassis_control.h"
#include "chassis_config.h"
#include "chassis_vars.h"
#include "encoder.h"
#include "motor_tb6612.h"
#include "pid.h"
#include "main.h"

static pid_ctrl_t s_pid[CHASSIS_WHEEL_COUNT];
static uint32_t s_last_step_ms;
static uint16_t s_stall_ms[CHASSIS_WHEEL_COUNT];
static uint8_t s_sensor_bad_count;
static bool s_initialized;
static bool s_motors_active;
static chassis_mode_t s_last_mode;

static float control_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float control_clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

static void control_stop_all(void)
{
    uint8_t i;

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        motor_coast(i);
        pid_reset(&s_pid[i]);
        (void)give_wheel_pwm(i, 0);
        s_stall_ms[i] = 0U;
    }

    s_sensor_bad_count = 0U;
    s_motors_active = false;
}

static void control_sample_encoders(float dt_s)
{
    uint8_t i;
    int16_t delta;
    float rpm;
    bool any_bad;
    var_status_t status;

    any_bad = false;

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        delta = encoder_read_delta(i);
        rpm = encoder_delta_to_rpm(delta, dt_s);
        status = give_measured_rpm(i, rpm);

        if (status != VAR_OK)
        {
            any_bad = true;
        }
    }

    if (any_bad)
    {
        if (s_sensor_bad_count < 255U)
        {
            s_sensor_bad_count++;
        }
    }
    else
    {
        s_sensor_bad_count = 0U;
    }
}

void chassis_control_init(void)
{
    chassis_snapshot_t snapshot;
    uint8_t i;

    vars_init();
    motor_init();
    encoder_init();

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        pid_init(&s_pid[i]);
        s_stall_ms[i] = 0U;
    }

    get_chassis_snapshot(&snapshot);
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        pid_configure(&s_pid[i],
                      snapshot.pid[i].kp,
                      snapshot.pid[i].ki,
                      snapshot.pid[i].kd);
    }

    s_last_step_ms = HAL_GetTick();
    s_sensor_bad_count = 0U;
    s_motors_active = false;
    s_last_mode = CHASSIS_MODE_IDLE;

    control_stop_all();
    s_initialized = true;
}

void chassis_control_update(uint32_t now_ms)
{
    chassis_snapshot_t snapshot;
    float dt_encoder_s;
    float dt_pid_s;
    float target_rpm[CHASSIS_WHEEL_COUNT];
    float left_mps;
    float right_mps;
    float rpm_per_mps;
    float feedforward;
    float pid_output;
    int16_t applied;
    uint32_t elapsed_ms;
    uint32_t command_age_ms;
    uint8_t i;
    var_status_t status;

    if (!s_initialized)
    {
        return;
    }

    elapsed_ms = now_ms - s_last_step_ms;
    if (elapsed_ms < CHASSIS_CONTROL_PERIOD_MS)
    {
        return;
    }
    s_last_step_ms = now_ms;

    dt_encoder_s = (float)elapsed_ms * 0.001f;
    dt_encoder_s = control_clamp(dt_encoder_s, 0.001f, 0.200f);
    dt_pid_s = control_clamp(dt_encoder_s, 0.005f, 0.050f);

    control_sample_encoders(dt_encoder_s);

    if (s_sensor_bad_count >= CHASSIS_SENSOR_FAULT_CYCLES)
    {
        (void)give_fault(FAULT_SENSOR);
    }

    get_chassis_snapshot(&snapshot);

    if ((snapshot.faults != 0U) || (!snapshot.enabled))
    {
        if (s_motors_active)
        {
            control_stop_all();
        }
        (void)give_control_cycle();
        return;
    }

    if (snapshot.mode == CHASSIS_MODE_IDLE)
    {
        if (s_motors_active)
        {
            control_stop_all();
        }
        (void)give_control_cycle();
        return;
    }

    command_age_ms = now_ms - snapshot.last_command_ms;
    if (command_age_ms > snapshot.command_timeout_ms)
    {
        (void)give_fault(FAULT_CMD_TIMEOUT);
        control_stop_all();
        (void)give_control_cycle();
        return;
    }

    if (snapshot.mode != s_last_mode)
    {
        for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
        {
            pid_reset(&s_pid[i]);
        }
        s_last_mode = snapshot.mode;
    }

    if (snapshot.mode == CHASSIS_MODE_VELOCITY)
    {
        rpm_per_mps = 60.0f / (CHASSIS_PI_F * CHASSIS_WHEEL_DIAMETER_M);
        left_mps = snapshot.linear_mps -
                   (snapshot.angular_radps * CHASSIS_TRACK_WIDTH_M * 0.5f);
        right_mps = snapshot.linear_mps +
                    (snapshot.angular_radps * CHASSIS_TRACK_WIDTH_M * 0.5f);

        target_rpm[CHASSIS_WHEEL_M1] = left_mps * rpm_per_mps;
        target_rpm[CHASSIS_WHEEL_M2] = right_mps * rpm_per_mps;
        target_rpm[CHASSIS_WHEEL_M3] = left_mps * rpm_per_mps;
        target_rpm[CHASSIS_WHEEL_M4] = right_mps * rpm_per_mps;

        status = give_resolved_targets(target_rpm);
        if (status != VAR_OK)
        {
            (void)give_fault(FAULT_CONFIG);
            control_stop_all();
            (void)give_control_cycle();
            return;
        }
    }
    else
    {
        for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
        {
            target_rpm[i] = snapshot.target_rpm[i];
        }
    }

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        pid_configure(&s_pid[i],
                      snapshot.pid[i].kp,
                      snapshot.pid[i].ki,
                      snapshot.pid[i].kd);

        /* 目标为 0 且轮子已经接近停住时直接滑行，避免积分把轮子反向带起来。 */
        if ((control_absf(target_rpm[i]) < 0.01f) &&
            (control_absf(snapshot.measured_rpm[i]) < CHASSIS_ZERO_HOLD_RPM))
        {
            motor_coast(i);
            pid_reset(&s_pid[i]);
            (void)give_wheel_pwm(i, 0);
            s_stall_ms[i] = 0U;
            continue;
        }

        feedforward = target_rpm[i] * CHASSIS_FEEDFORWARD_PERMILLE_PER_RPM;
        feedforward = control_clamp(feedforward,
                                    -(float)CHASSIS_PWM_MAX,
                                    (float)CHASSIS_PWM_MAX);

        pid_output = pid_update(&s_pid[i],
                                target_rpm[i],
                                snapshot.measured_rpm[i],
                                feedforward,
                                dt_pid_s);

        applied = motor_set_output(i,
                                   (int16_t)((pid_output >= 0.0f)
                                             ? (pid_output + 0.5f)
                                             : (pid_output - 0.5f)),
                                   now_ms);

        status = give_wheel_pwm(i, applied);
        if (status != VAR_OK)
        {
            (void)give_fault(FAULT_CONFIG);
            control_stop_all();
            (void)give_control_cycle();
            return;
        }

        /* 堵转检测：目标足够大、输出足够大，但轮速持续接近 0 或反向。 */
        if ((control_absf(target_rpm[i]) >= CHASSIS_STALL_TARGET_RPM) &&
            (control_absf((float)applied) >= (float)CHASSIS_STALL_PWM_MIN))
        {
            if ((control_absf(snapshot.measured_rpm[i]) <
                 CHASSIS_STALL_MEASURED_RPM) ||
                ((snapshot.measured_rpm[i] * target_rpm[i]) < 0.0f))
            {
                if (s_stall_ms[i] <= (uint16_t)(0xFFFFU - elapsed_ms))
                {
                    s_stall_ms[i] = (uint16_t)(s_stall_ms[i] + elapsed_ms);
                }
            }
            else
            {
                s_stall_ms[i] = 0U;
            }
        }
        else
        {
            s_stall_ms[i] = 0U;
        }

        if (s_stall_ms[i] >= CHASSIS_STALL_TIME_MS)
        {
            (void)give_fault((uint16_t)(FAULT_STALL_M1 << i));
            control_stop_all();
            (void)give_control_cycle();
            return;
        }
    }

    s_motors_active = true;
    (void)give_control_cycle();
}
