/**
  ******************************************************************************
  * @file    motor_tb6612.c
  * @brief   TB6612 四路电机底层驱动。
  *
  * 接线（来自接线图）：
  *   M1: PWM PE9/TIM1_CH1, AIN1 PA5, AIN2 PA4
  *   M2: PWM PE11/TIM1_CH2, BIN1 PB14, BIN2 PB15
  *   M3: PWM PE13/TIM1_CH3, AIN1 PD9, AIN2 PD8
  *   M4: PWM PE14/TIM1_CH4, BIN1 PD10, BIN2 PD11
  ******************************************************************************
  */
#include "motor_tb6612.h"
#include "chassis_config.h"
#include "tim.h"
#include "main.h"

typedef struct
{
    uint32_t channel;
    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;
    int8_t sign;
} motor_hw_t;

static const motor_hw_t s_motor_hw[CHASSIS_WHEEL_COUNT] =
{
    { TIM_CHANNEL_1, GPIOA, GPIO_PIN_5,  GPIOA, GPIO_PIN_4,  CHASSIS_MOTOR_SIGN_M1 },
    { TIM_CHANNEL_2, GPIOB, GPIO_PIN_14, GPIOB, GPIO_PIN_15, CHASSIS_MOTOR_SIGN_M2 },
    { TIM_CHANNEL_3, GPIOD, GPIO_PIN_9,  GPIOD, GPIO_PIN_8,  CHASSIS_MOTOR_SIGN_M3 },
    { TIM_CHANNEL_4, GPIOD, GPIO_PIN_10, GPIOD, GPIO_PIN_11, CHASSIS_MOTOR_SIGN_M4 }
};

/* 仅 motor_set_output() 使用的换向/斜率状态。 */
static int8_t s_last_dir[CHASSIS_WHEEL_COUNT];
static uint16_t s_last_duty[CHASSIS_WHEEL_COUNT];
static uint32_t s_deadtime_until[CHASSIS_WHEEL_COUNT];

static void motor_reset_tracking(uint8_t motor)
{
    s_last_dir[motor] = 0;
    s_last_duty[motor] = 0U;
    s_deadtime_until[motor] = 0U;
}

static void motor_write_pwm(const motor_hw_t *hw, uint16_t permille)
{
    uint32_t pulse;

    if (permille > CHASSIS_PWM_MAX)
    {
        permille = CHASSIS_PWM_MAX;
    }

    pulse = ((uint32_t)permille * htim1.Init.Period) / CHASSIS_PWM_MAX;
    __HAL_TIM_SET_COMPARE(&htim1, hw->channel, pulse);
}

static void motor_write_pins(const motor_hw_t *hw,
                             GPIO_PinState in1,
                             GPIO_PinState in2)
{
    HAL_GPIO_WritePin(hw->in1_port, hw->in1_pin, in1);
    HAL_GPIO_WritePin(hw->in2_port, hw->in2_pin, in2);
}

/* 只操作硬件，不改换向/斜率状态。 */
static void motor_coast_hw(uint8_t motor)
{
    motor_write_pins(&s_motor_hw[motor], GPIO_PIN_RESET, GPIO_PIN_RESET);
    motor_write_pwm(&s_motor_hw[motor], 0U);
}

/*---------------------------------------------------------------------------*/
/* 公共操作                                                                    */
/*---------------------------------------------------------------------------*/
void motor_init(void)
{
    uint8_t i;

    /* 统一在这里落到 20 kHz：即使 CubeMX 重新生成 tim.c 回到旧 ARR，
     * 电机驱动初始化时也会覆盖，占空比换算同时使用新的 ARR。 */
    htim1.Init.Prescaler = 0U;
    htim1.Init.Period = CHASSIS_MOTOR_PWM_ARR;
    __HAL_TIM_SET_PRESCALER(&htim1, 0U);
    __HAL_TIM_SET_AUTORELOAD(&htim1, CHASSIS_MOTOR_PWM_ARR);

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        motor_reset_tracking(i);
        motor_write_pins(&s_motor_hw[i], GPIO_PIN_RESET, GPIO_PIN_RESET);
        motor_write_pwm(&s_motor_hw[i], 0U);
        (void)HAL_TIM_PWM_Start(&htim1, s_motor_hw[i].channel);
    }
}

void motor_coast(uint8_t motor)
{
    if (motor >= CHASSIS_WHEEL_COUNT)
    {
        return;
    }

    /* 两个方向脚都为低，PWM 为 0：电机两端悬空，自由滑行。 */
    motor_coast_hw(motor);
    motor_reset_tracking(motor);
}

void motor_brake(uint8_t motor)
{
    if (motor >= CHASSIS_WHEEL_COUNT)
    {
        return;
    }

    /* TB6612 短接制动：IN1=IN2=1，同时 PWM 给满。 */
    motor_write_pwm(&s_motor_hw[motor], 0U);
    motor_write_pins(&s_motor_hw[motor], GPIO_PIN_SET, GPIO_PIN_SET);
    motor_write_pwm(&s_motor_hw[motor], CHASSIS_PWM_MAX);
    motor_reset_tracking(motor);
}

void motor_forward(uint8_t motor, uint16_t permille)
{
    if (motor >= CHASSIS_WHEEL_COUNT)
    {
        return;
    }

    if (permille > CHASSIS_PWM_MAX)
    {
        permille = CHASSIS_PWM_MAX;
    }

    motor_write_pwm(&s_motor_hw[motor], 0U);
    motor_write_pins(&s_motor_hw[motor], GPIO_PIN_SET, GPIO_PIN_RESET);
    motor_write_pwm(&s_motor_hw[motor], permille);
    motor_reset_tracking(motor);
}

void motor_reverse(uint8_t motor, uint16_t permille)
{
    if (motor >= CHASSIS_WHEEL_COUNT)
    {
        return;
    }

    if (permille > CHASSIS_PWM_MAX)
    {
        permille = CHASSIS_PWM_MAX;
    }

    motor_write_pwm(&s_motor_hw[motor], 0U);
    motor_write_pins(&s_motor_hw[motor], GPIO_PIN_RESET, GPIO_PIN_SET);
    motor_write_pwm(&s_motor_hw[motor], permille);
    motor_reset_tracking(motor);
}

int16_t motor_set_output(uint8_t motor, int16_t permille, uint32_t now_ms)
{
    int8_t logical_dir;
    int8_t hardware_dir;
    uint16_t duty;
    uint16_t next_duty;

    if (motor >= CHASSIS_WHEEL_COUNT)
    {
        return 0;
    }

    if (permille > CHASSIS_PWM_MAX)
    {
        permille = CHASSIS_PWM_MAX;
    }
    else if (permille < -CHASSIS_PWM_MAX)
    {
        permille = -CHASSIS_PWM_MAX;
    }

    if (permille == 0)
    {
        if (s_last_dir[motor] != 0)
        {
            /* 从有方向输出变为 0：给后续换向留出电流衰减时间。 */
            motor_coast_hw(motor);
            s_last_dir[motor] = 0;
            s_last_duty[motor] = 0U;
            s_deadtime_until[motor] =
                now_ms + CHASSIS_MOTOR_REVERSE_DEADTIME_MS;
        }
        else
        {
            motor_coast_hw(motor);
        }
        return 0;
    }

    logical_dir = (permille > 0) ? 1 : -1;
    duty = (uint16_t)((permille > 0) ? permille : -permille);

    /* 换向：先滑行，等待电流衰减后再反向。 */
    if ((s_last_dir[motor] != 0) && (s_last_dir[motor] != logical_dir))
    {
        motor_coast_hw(motor);
        s_last_dir[motor] = 0;
        s_last_duty[motor] = 0U;
        s_deadtime_until[motor] = now_ms + CHASSIS_MOTOR_REVERSE_DEADTIME_MS;
        return 0;
    }

    if ((s_last_dir[motor] == 0) &&
        ((int32_t)(now_ms - s_deadtime_until[motor]) < 0))
    {
        motor_coast_hw(motor);
        return 0;
    }

    /* 占空比斜率限制，减小启动电流冲击。 */
    if (s_last_dir[motor] == logical_dir)
    {
        if (duty > (uint16_t)(s_last_duty[motor] + CHASSIS_MOTOR_SLEW_PERMILLE))
        {
            next_duty = (uint16_t)(s_last_duty[motor] + CHASSIS_MOTOR_SLEW_PERMILLE);
        }
        else if ((uint32_t)duty + CHASSIS_MOTOR_SLEW_PERMILLE < s_last_duty[motor])
        {
            next_duty = (uint16_t)(s_last_duty[motor] - CHASSIS_MOTOR_SLEW_PERMILLE);
        }
        else
        {
            next_duty = duty;
        }
    }
    else
    {
        next_duty = (duty > CHASSIS_MOTOR_SLEW_PERMILLE)
                  ? CHASSIS_MOTOR_SLEW_PERMILLE
                  : duty;
    }

    hardware_dir = (int8_t)(logical_dir * s_motor_hw[motor].sign);

    if (hardware_dir > 0)
    {
        motor_write_pins(&s_motor_hw[motor], GPIO_PIN_SET, GPIO_PIN_RESET);
    }
    else
    {
        motor_write_pins(&s_motor_hw[motor], GPIO_PIN_RESET, GPIO_PIN_SET);
    }
    motor_write_pwm(&s_motor_hw[motor], next_duty);

    s_last_dir[motor] = logical_dir;
    s_last_duty[motor] = next_duty;

    return (int16_t)((logical_dir > 0) ? next_duty : -next_duty);
}
