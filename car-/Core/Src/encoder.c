/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   四路增量编码器测速。
  *
  * 接线（来自接线图）：
  *   M1: A=PD12(TIM4_CH1) B=PD13(TIM4_CH2)
  *   M2: A=PA15(TIM2_CH1) B=PB3 (TIM2_CH2, 部分重映射)
  *   M3: A=PA6 (TIM3_CH1) B=PA7 (TIM3_CH2)
  *   M4: A=PA0 (TIM5_CH1) B=PA1 (TIM5_CH2)
  ******************************************************************************
  */
#include "encoder.h"
#include "chassis_config.h"
#include "tim.h"
#include "main.h"

typedef struct
{
    TIM_HandleTypeDef *timer;
    GPIO_TypeDef *port_a;
    uint16_t pin_a;
    GPIO_TypeDef *port_b;
    uint16_t pin_b;
    int8_t sign;
} encoder_hw_t;

static const encoder_hw_t s_encoder_hw[CHASSIS_WHEEL_COUNT] =
{
    { &htim4, GPIOD, GPIO_PIN_12, GPIOD, GPIO_PIN_13, CHASSIS_ENCODER_SIGN_M1 },
    { &htim2, GPIOA, GPIO_PIN_15, GPIOB, GPIO_PIN_3,  CHASSIS_ENCODER_SIGN_M2 },
    { &htim3, GPIOA, GPIO_PIN_6,  GPIOA, GPIO_PIN_7,  CHASSIS_ENCODER_SIGN_M3 },
    { &htim5, GPIOA, GPIO_PIN_0,  GPIOA, GPIO_PIN_1,  CHASSIS_ENCODER_SIGN_M4 }
};

static uint16_t s_last_count[CHASSIS_WHEEL_COUNT];

void encoder_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t i;

    /* PA15/PB3 默认是 JTAG 引脚，必须只保留 SWD，TIM2 才能使用这两个脚。 */
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        TIM_TypeDef *instance = s_encoder_hw[i].timer->Instance;

        /* CubeMX 里 TIM2 还附带了 One-Pulse 虚拟模式，会置 CR1.OPM，
         * 导致编码器计数到一次更新事件后停止，这里强制清掉。 */
        CLEAR_BIT(instance->CR1, TIM_CR1_OPM);

        /* CubeMX 默认 TI1 两倍频，这里改为 TI1+TI2 四倍频。 */
        MODIFY_REG(instance->SMCR, TIM_SMCR_SMS, TIM_ENCODERMODE_TI12);

        /* 输入滤波 6 个采样周期，抑制电机换向噪声。 */
        MODIFY_REG(instance->CCMR1,
                   TIM_CCMR1_IC1F | TIM_CCMR1_IC2F,
                   (6UL << 4) | (6UL << 12));

        gpio.Pin = s_encoder_hw[i].pin_a;
        HAL_GPIO_Init(s_encoder_hw[i].port_a, &gpio);
        gpio.Pin = s_encoder_hw[i].pin_b;
        HAL_GPIO_Init(s_encoder_hw[i].port_b, &gpio);

        (void)HAL_TIM_Encoder_Start(s_encoder_hw[i].timer, TIM_CHANNEL_ALL);
        __HAL_TIM_SET_COUNTER(s_encoder_hw[i].timer, 0U);
        s_last_count[i] = 0U;
    }
}

void encoder_reset(void)
{
    uint8_t i;

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        __HAL_TIM_SET_COUNTER(s_encoder_hw[i].timer, 0U);
        s_last_count[i] = 0U;
    }
}

int16_t encoder_read_delta(uint8_t wheel)
{
    uint16_t now_count;
    uint16_t diff;
    int32_t delta;

    if (wheel >= CHASSIS_WHEEL_COUNT)
    {
        return 0;
    }

    now_count = (uint16_t)__HAL_TIM_GET_COUNTER(s_encoder_hw[wheel].timer);
    diff = (uint16_t)(now_count - s_last_count[wheel]);
    s_last_count[wheel] = now_count;

    /* 16 位计数器回绕展开。 */
    delta = ((diff & 0x8000U) != 0U) ? ((int32_t)diff - 65536L) : (int32_t)diff;
    return (int16_t)(delta * (int32_t)s_encoder_hw[wheel].sign);
}

float encoder_delta_to_rpm(int16_t delta, float dt_s)
{
    if (dt_s <= 0.0f)
    {
        return 0.0f;
    }

    return ((float)delta * 60.0f) / (CHASSIS_COUNTS_PER_WHEEL_REV * dt_s);
}
