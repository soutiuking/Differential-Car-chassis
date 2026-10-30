/**
  ******************************************************************************
  * @file    chassis_vars.c
  * @brief   底盘运行期变量的唯一存储与校验实现。
  ******************************************************************************
  */
#include "chassis_vars.h"
#include "main.h"

/* 唯一一份运行期变量。static 防止其他模块通过 extern 直接改写。 */
static chassis_snapshot_t s_vars;

/*---------------------------------------------------------------------------*/
/* 临界区：保证多字段读写的一致性                                               */
/*---------------------------------------------------------------------------*/
static uint32_t vars_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void vars_exit_critical(uint32_t primask)
{
    if ((primask & 1U) == 0U)
    {
        __enable_irq();
    }
}

/*---------------------------------------------------------------------------*/
/* 内部校验函数：只允许被 give_* 调用                                           */
/*---------------------------------------------------------------------------*/
static bool vars_wheel_valid(uint8_t wheel)
{
    return wheel < CHASSIS_WHEEL_COUNT;
}

/* NaN 与 inf 都会使区间比较为假，因此这里同时完成有限值检查。 */
static bool vars_in_range(float value, float min_value, float max_value)
{
    return (value >= min_value) && (value <= max_value);
}

static bool vars_symmetric_range(float value, float limit)
{
    return (value >= -limit) && (value <= limit);
}

/*---------------------------------------------------------------------------*/
/* 初始化                                                                      */
/*---------------------------------------------------------------------------*/
void vars_init(void)
{
    uint32_t primask;
    uint8_t i;

    primask = vars_enter_critical();

    s_vars.mode = CHASSIS_MODE_IDLE;
    s_vars.enabled = false;
    s_vars.faults = 0U;
    s_vars.linear_mps = 0.0f;
    s_vars.angular_radps = 0.0f;
    s_vars.command_timeout_ms = CHASSIS_COMMAND_TIMEOUT_DEFAULT_MS;
    s_vars.last_command_ms = 0U;

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        s_vars.target_rpm[i] = 0.0f;
        s_vars.measured_rpm[i] = 0.0f;
        s_vars.pwm_permille[i] = 0;
        s_vars.pid[i].kp = CHASSIS_PID_KP_DEFAULT;
        s_vars.pid[i].ki = CHASSIS_PID_KI_DEFAULT;
        s_vars.pid[i].kd = CHASSIS_PID_KD_DEFAULT;
    }

    s_vars.rx_frames_ok = 0U;
    s_vars.rx_frames_bad = 0U;
    s_vars.control_cycles = 0U;

    vars_exit_critical(primask);
}

/*---------------------------------------------------------------------------*/
/* get_*                                                                       */
/*---------------------------------------------------------------------------*/
chassis_mode_t get_chassis_mode(void)
{
    uint32_t primask = vars_enter_critical();
    chassis_mode_t value = s_vars.mode;
    vars_exit_critical(primask);
    return value;
}

bool get_chassis_enabled(void)
{
    uint32_t primask = vars_enter_critical();
    bool value = s_vars.enabled;
    vars_exit_critical(primask);
    return value;
}

uint16_t get_chassis_faults(void)
{
    uint32_t primask = vars_enter_critical();
    uint16_t value = s_vars.faults;
    vars_exit_critical(primask);
    return value;
}

float get_linear_command_mps(void)
{
    uint32_t primask = vars_enter_critical();
    float value = s_vars.linear_mps;
    vars_exit_critical(primask);
    return value;
}

float get_angular_command_radps(void)
{
    uint32_t primask = vars_enter_critical();
    float value = s_vars.angular_radps;
    vars_exit_critical(primask);
    return value;
}

uint32_t get_command_timeout_ms(void)
{
    uint32_t primask = vars_enter_critical();
    uint32_t value = s_vars.command_timeout_ms;
    vars_exit_critical(primask);
    return value;
}

uint32_t get_last_command_ms(void)
{
    uint32_t primask = vars_enter_critical();
    uint32_t value = s_vars.last_command_ms;
    vars_exit_critical(primask);
    return value;
}

float get_wheel_target_rpm(uint8_t wheel)
{
    float value = 0.0f;
    uint32_t primask;

    if (vars_wheel_valid(wheel))
    {
        primask = vars_enter_critical();
        value = s_vars.target_rpm[wheel];
        vars_exit_critical(primask);
    }
    return value;
}

float get_wheel_measured_rpm(uint8_t wheel)
{
    float value = 0.0f;
    uint32_t primask;

    if (vars_wheel_valid(wheel))
    {
        primask = vars_enter_critical();
        value = s_vars.measured_rpm[wheel];
        vars_exit_critical(primask);
    }
    return value;
}

int16_t get_wheel_pwm_permille(uint8_t wheel)
{
    int16_t value = 0;
    uint32_t primask;

    if (vars_wheel_valid(wheel))
    {
        primask = vars_enter_critical();
        value = s_vars.pwm_permille[wheel];
        vars_exit_critical(primask);
    }
    return value;
}

void get_pid_gains(uint8_t wheel, pid_gains_t *out)
{
    uint32_t primask;

    if ((out == NULL) || (!vars_wheel_valid(wheel)))
    {
        return;
    }

    primask = vars_enter_critical();
    *out = s_vars.pid[wheel];
    vars_exit_critical(primask);
}

uint32_t get_rx_frames_ok(void)
{
    uint32_t primask = vars_enter_critical();
    uint32_t value = s_vars.rx_frames_ok;
    vars_exit_critical(primask);
    return value;
}

uint32_t get_rx_frames_bad(void)
{
    uint32_t primask = vars_enter_critical();
    uint32_t value = s_vars.rx_frames_bad;
    vars_exit_critical(primask);
    return value;
}

uint32_t get_control_cycles(void)
{
    uint32_t primask = vars_enter_critical();
    uint32_t value = s_vars.control_cycles;
    vars_exit_critical(primask);
    return value;
}

void get_chassis_snapshot(chassis_snapshot_t *out)
{
    uint32_t primask;

    if (out == NULL)
    {
        return;
    }

    primask = vars_enter_critical();
    *out = s_vars;
    vars_exit_critical(primask);
}

/*---------------------------------------------------------------------------*/
/* give_*                                                                      */
/*---------------------------------------------------------------------------*/
var_status_t give_motion_command(float linear_mps,
                                 float angular_radps,
                                 uint32_t timeout_ms,
                                 uint32_t now_ms)
{
    uint32_t primask;
    var_status_t status = VAR_OK;

    primask = vars_enter_critical();

    if (!vars_symmetric_range(linear_mps, CHASSIS_MAX_LINEAR_MPS) ||
        !vars_symmetric_range(angular_radps, CHASSIS_MAX_ANGULAR_RADPS) ||
        (timeout_ms < CHASSIS_COMMAND_TIMEOUT_MIN_MS) ||
        (timeout_ms > CHASSIS_COMMAND_TIMEOUT_MAX_MS))
    {
        status = VAR_ERR_RANGE;
    }
    else if (!s_vars.enabled || (s_vars.faults != 0U))
    {
        status = VAR_ERR_STATE;
    }
    else
    {
        s_vars.linear_mps = linear_mps;
        s_vars.angular_radps = angular_radps;
        s_vars.command_timeout_ms = timeout_ms;
        s_vars.last_command_ms = now_ms;
        s_vars.mode = CHASSIS_MODE_VELOCITY;
    }

    vars_exit_critical(primask);
    return status;
}

var_status_t give_wheel_command(const float rpm[CHASSIS_WHEEL_COUNT],
                                uint32_t now_ms)
{
    uint32_t primask;
    var_status_t status = VAR_OK;
    uint8_t i;

    if (rpm == NULL)
    {
        return VAR_ERR_ARG;
    }

    primask = vars_enter_critical();

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        if (!vars_symmetric_range(rpm[i], CHASSIS_MAX_WHEEL_RPM))
        {
            status = VAR_ERR_RANGE;
            break;
        }
    }

    if ((status == VAR_OK) && (!s_vars.enabled || (s_vars.faults != 0U)))
    {
        status = VAR_ERR_STATE;
    }

    if (status == VAR_OK)
    {
        for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
        {
            s_vars.target_rpm[i] = rpm[i];
        }
        s_vars.linear_mps = 0.0f;
        s_vars.angular_radps = 0.0f;
        s_vars.command_timeout_ms = CHASSIS_COMMAND_TIMEOUT_DEFAULT_MS;
        s_vars.last_command_ms = now_ms;
        s_vars.mode = CHASSIS_MODE_WHEEL_RPM;
    }

    vars_exit_critical(primask);
    return status;
}

var_status_t give_resolved_targets(const float rpm[CHASSIS_WHEEL_COUNT])
{
    uint32_t primask;
    var_status_t status = VAR_OK;
    uint8_t i;

    if (rpm == NULL)
    {
        return VAR_ERR_ARG;
    }

    primask = vars_enter_critical();

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        if (!vars_symmetric_range(rpm[i], CHASSIS_MAX_WHEEL_RPM))
        {
            status = VAR_ERR_RANGE;
            break;
        }
    }

    if (status == VAR_OK)
    {
        for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
        {
            s_vars.target_rpm[i] = rpm[i];
        }
    }

    vars_exit_critical(primask);
    return status;
}

var_status_t give_measured_rpm(uint8_t wheel, float rpm)
{
    uint32_t primask;
    var_status_t status = VAR_OK;

    primask = vars_enter_critical();

    if (!vars_wheel_valid(wheel))
    {
        status = VAR_ERR_ARG;
    }
    else if (!vars_symmetric_range(rpm, CHASSIS_MAX_MEASURED_RPM))
    {
        /* 测量值异常时仍写入限幅值，保证 PID 不会读到上一次的陈旧速度；
         * 连续多次异常由控制层判为传感器故障。 */
        if (rpm != rpm)
        {
            s_vars.measured_rpm[wheel] = 0.0f;
        }
        else
        {
            s_vars.measured_rpm[wheel] =
                (rpm > 0.0f) ? CHASSIS_MAX_MEASURED_RPM : -CHASSIS_MAX_MEASURED_RPM;
        }
        status = VAR_ERR_RANGE;
    }
    else
    {
        s_vars.measured_rpm[wheel] = rpm;
    }

    vars_exit_critical(primask);
    return status;
}

var_status_t give_wheel_pwm(uint8_t wheel, int16_t permille)
{
    uint32_t primask;
    var_status_t status = VAR_OK;

    primask = vars_enter_critical();

    if (!vars_wheel_valid(wheel))
    {
        status = VAR_ERR_ARG;
    }
    else if ((permille < -CHASSIS_PWM_MAX) || (permille > CHASSIS_PWM_MAX))
    {
        status = VAR_ERR_RANGE;
    }
    else
    {
        s_vars.pwm_permille[wheel] = permille;
    }

    vars_exit_critical(primask);
    return status;
}

var_status_t give_pid_gains(uint8_t wheel, float kp, float ki, float kd)
{
    uint32_t primask;
    var_status_t status = VAR_OK;

    primask = vars_enter_critical();

    if (!vars_wheel_valid(wheel))
    {
        status = VAR_ERR_ARG;
    }
    else if (!vars_in_range(kp, 0.0f, CHASSIS_PID_KP_MAX) ||
             !vars_in_range(ki, 0.0f, CHASSIS_PID_KI_MAX) ||
             !vars_in_range(kd, 0.0f, CHASSIS_PID_KD_MAX))
    {
        status = VAR_ERR_RANGE;
    }
    else
    {
        s_vars.pid[wheel].kp = kp;
        s_vars.pid[wheel].ki = ki;
        s_vars.pid[wheel].kd = kd;
    }

    vars_exit_critical(primask);
    return status;
}

var_status_t give_enable(bool enable, bool clear_faults, uint32_t now_ms)
{
    uint32_t primask;
    var_status_t status = VAR_OK;
    uint8_t i;

    primask = vars_enter_critical();

    if (clear_faults)
    {
        s_vars.faults = 0U;
    }

    if (enable)
    {
        if (s_vars.faults != 0U)
        {
            status = VAR_ERR_STATE;
        }
        else
        {
            s_vars.enabled = true;
            s_vars.mode = CHASSIS_MODE_IDLE;
            s_vars.linear_mps = 0.0f;
            s_vars.angular_radps = 0.0f;
            for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
            {
                s_vars.target_rpm[i] = 0.0f;
            }
            s_vars.last_command_ms = now_ms;
        }
    }
    else
    {
        s_vars.enabled = false;
        s_vars.mode = CHASSIS_MODE_IDLE;
        s_vars.linear_mps = 0.0f;
        s_vars.angular_radps = 0.0f;
        for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
        {
            s_vars.target_rpm[i] = 0.0f;
        }
    }

    vars_exit_critical(primask);
    return status;
}

var_status_t give_fault(uint16_t fault_mask)
{
    uint32_t primask;
    var_status_t status = VAR_OK;
    uint8_t i;

    primask = vars_enter_critical();

    if (fault_mask == 0U)
    {
        status = VAR_ERR_ARG;
    }
    else if ((fault_mask & (uint16_t)(~FAULT_ALL_MASK)) != 0U)
    {
        status = VAR_ERR_RANGE;
    }
    else
    {
        s_vars.faults |= fault_mask;
        /* 任何故障都立即退出使能并清空目标。 */
        s_vars.enabled = false;
        s_vars.mode = CHASSIS_MODE_IDLE;
        s_vars.linear_mps = 0.0f;
        s_vars.angular_radps = 0.0f;
        for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
        {
            s_vars.target_rpm[i] = 0.0f;
        }
    }

    vars_exit_critical(primask);
    return status;
}

var_status_t give_clear_faults(void)
{
    uint32_t primask;

    primask = vars_enter_critical();
    s_vars.faults = 0U;
    vars_exit_critical(primask);

    return VAR_OK;
}

var_status_t give_comm_result(bool frame_ok)
{
    uint32_t primask;

    primask = vars_enter_critical();
    if (frame_ok)
    {
        if (s_vars.rx_frames_ok != 0xFFFFFFFFU)
        {
            s_vars.rx_frames_ok++;
        }
    }
    else
    {
        if (s_vars.rx_frames_bad != 0xFFFFFFFFU)
        {
            s_vars.rx_frames_bad++;
        }
    }
    vars_exit_critical(primask);

    return VAR_OK;
}

var_status_t give_control_cycle(void)
{
    uint32_t primask;

    primask = vars_enter_critical();
    if (s_vars.control_cycles != 0xFFFFFFFFU)
    {
        s_vars.control_cycles++;
    }
    vars_exit_critical(primask);

    return VAR_OK;
}
