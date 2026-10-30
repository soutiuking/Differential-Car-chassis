/**
  ******************************************************************************
  * @file    board_ui.c
  * @brief   PC0~PC3 共阳极 LED，PC4~PC7 用户按键。
  *
  * 按键：
  *   K1 PC4：使能 / 断使能
  *   K2 PC5：日常模式 / 比赛模式
  *   K3 PC6：MPU6050 零偏标定（先停车、断使能）
  *   K4 PC7：OLED 正常页 / 调试页
  *
  * LED（共阳极，低电平点亮）：
  *   PC0：心跳
  *   PC1：已使能
  *   PC2：故障/标定结果
  *   PC3：OLED 调试页
  ******************************************************************************
  */
#include "board_ui.h"
#include "main.h"
#include "chassis_vars.h"
#include "motion_control.h"
#include "nav_ekf.h"
#include "nav_task.h"
#include "nav_vars.h"
#include "mpu6050.h"
#include "param_store.h"

#define BTN_PORT          GPIOC
#define BTN_K1_PIN        GPIO_PIN_4
#define BTN_K2_PIN        GPIO_PIN_5
#define BTN_K3_PIN        GPIO_PIN_6
#define BTN_K4_PIN        GPIO_PIN_7
#define LED_PORT          GPIOC
#define LED_ALL_PINS      (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3)
#define BTN_DEBOUNCE      3U
#define UI_PERIOD_MS      10U

static const uint16_t s_btn_pin[4] =
{
    BTN_K1_PIN, BTN_K2_PIN, BTN_K3_PIN, BTN_K4_PIN
};

static uint8_t s_btn_raw[4];
static uint8_t s_btn_count[4];
static uint8_t s_btn_stable[4];
static uint32_t s_last_ui_ms;
static uint32_t s_heartbeat_ms;
static bool s_heartbeat_on;
static uint32_t s_calib_result_until_ms;
static bool s_calib_result_ok;

static void led_write(uint8_t index, bool on)
{
    if (index > 3U)
    {
        return;
    }
    HAL_GPIO_WritePin(LED_PORT, (uint16_t)(1U << index),
                      on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void board_ui_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* 共阳极 LED：先输出高电平，保证上电熄灭。 */
    HAL_GPIO_WritePin(LED_PORT, LED_ALL_PINS, GPIO_PIN_SET);
    gpio.Pin = LED_ALL_PINS;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gpio);

    /* 按键低电平有效，使用内部上拉。 */
    gpio.Pin = BTN_K1_PIN | BTN_K2_PIN | BTN_K3_PIN | BTN_K4_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BTN_PORT, &gpio);

    s_last_ui_ms = HAL_GetTick();
    s_heartbeat_ms = s_last_ui_ms;
    s_heartbeat_on = false;
    s_calib_result_until_ms = 0U;
    s_calib_result_ok = false;

    nav_display_set_mode(NAV_DISPLAY_NORMAL);
}

static void button_k1(uint32_t now_ms)
{
    if (get_chassis_enabled())
    {
        motion_abort(now_ms);
        (void)give_enable(false, false, now_ms);
    }
    else
    {
        motion_abort(now_ms);
        (void)give_enable(true, true, now_ms);
    }
}

static void button_k2(void)
{
    uint8_t mode = (get_nav_mode() == 0U) ? 1U : 0U;
    if (give_nav_mode(mode) == VAR_OK)
    {
        (void)param_store_set_value(PARAM_ID_NAV_MODE, (int32_t)mode);
    }
}

static void button_k3(uint32_t now_ms)
{
    bool ok;

    /* 标定期间必须完全静止：先停路径、断使能。 */
    motion_abort(now_ms);
    (void)give_enable(false, false, now_ms);

    nav_display_set_mode(NAV_DISPLAY_CALIB);
    nav_display_refresh_now();
    led_write(2U, true);

    ok = mpu6050_calibrate_gyro(200U);
    if (ok)
    {
        /* 驱动已经扣除静态零偏，EKF 残余 bias 从 0 重新开始。 */
        nav_ekf_set_gyro_bias(0.0f);
    }

    s_calib_result_until_ms = HAL_GetTick() + 2000U;
    s_calib_result_ok = ok;
    led_write(2U, ok ? false : true);

    nav_display_set_mode(NAV_DISPLAY_NORMAL);
    nav_display_refresh_now();
}

static void button_k4(void)
{
    if (nav_display_get_mode() == NAV_DISPLAY_DEBUG)
    {
        nav_display_set_mode(NAV_DISPLAY_NORMAL);
    }
    else
    {
        nav_display_set_mode(NAV_DISPLAY_DEBUG);
    }
}

static void button_event(uint8_t index, uint32_t now_ms)
{
    switch (index)
    {
    case 0U:
        button_k1(now_ms);
        break;
    case 1U:
        button_k2();
        break;
    case 2U:
        button_k3(now_ms);
        break;
    case 3U:
        button_k4();
        break;
    default:
        break;
    }
}

static void button_scan(uint32_t now_ms)
{
    uint8_t i;

    for (i = 0U; i < 4U; i++)
    {
        uint8_t raw = (HAL_GPIO_ReadPin(BTN_PORT, s_btn_pin[i]) == GPIO_PIN_RESET)
                    ? 1U : 0U;

        if (raw != s_btn_raw[i])
        {
            s_btn_raw[i] = raw;
            s_btn_count[i] = 0U;
        }
        else if (s_btn_count[i] < BTN_DEBOUNCE)
        {
            s_btn_count[i]++;
        }

        if ((s_btn_count[i] >= BTN_DEBOUNCE) &&
            (s_btn_raw[i] != 0U) && (s_btn_stable[i] == 0U))
        {
            s_btn_stable[i] = 1U;
            button_event(i, now_ms);
        }
        else if ((s_btn_count[i] >= BTN_DEBOUNCE) && (s_btn_raw[i] == 0U))
        {
            s_btn_stable[i] = 0U;
        }
    }
}

void board_ui_update(uint32_t now_ms)
{
    uint8_t faults;

    if ((uint32_t)(now_ms - s_last_ui_ms) < UI_PERIOD_MS)
    {
        return;
    }
    s_last_ui_ms = now_ms;

    button_scan(now_ms);

    if ((uint32_t)(now_ms - s_heartbeat_ms) >= 500U)
    {
        s_heartbeat_ms = now_ms;
        s_heartbeat_on = !s_heartbeat_on;
    }

    led_write(0U, s_heartbeat_on);
    led_write(1U, get_chassis_enabled());

    faults = (get_chassis_faults() != 0U) ? 1U : 0U;
    if (faults != 0U)
    {
        led_write(2U, true);
    }
    else if ((int32_t)(now_ms - s_calib_result_until_ms) < 0)
    {
        /* 标定结束后闪 2 秒：成功灭，失败闪。 */
        led_write(2U, s_calib_result_ok ? false : ((now_ms / 100U) & 1U) != 0U);
    }
    else
    {
        led_write(2U, false);
    }

    led_write(3U, nav_display_get_mode() == NAV_DISPLAY_DEBUG);
}
