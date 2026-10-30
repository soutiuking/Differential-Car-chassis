/**
  ******************************************************************************
  * @file    tof400.c
  * @brief   TOF400C（VL53L1X）双传感器初始化与读取。
  ******************************************************************************
  */
#include "tof400.h"
#include "VL53L1X_api.h"
#include "main.h"
#include <math.h>

#define TOF_DEFAULT_ADDR        (0x29U << 1)   /* 0x52，8 位地址 */
#define TOF_FRONT_ADDR          (0x30U << 1)   /* 0x60 */
#define TOF_RIGHT_ADDR          (0x31U << 1)   /* 0x62 */

#define TOF_XSHUT_PORT          GPIOB
#define TOF_FRONT_XSHUT_PIN     GPIO_PIN_12
#define TOF_RIGHT_XSHUT_PIN     GPIO_PIN_13

#define TOF_TIMING_BUDGET_MS    100U
#define TOF_INTER_MEAS_MS       250U
#define TOF_PHASE_OFFSET_MS     125U
#define TOF_RANGE_MIN_MM        20U
#define TOF_RANGE_MAX_MM        4000U
#define TOF_RANGE_STATUS_MAX    2U

typedef struct
{
    uint8_t address;
    uint16_t xshut_pin;
    bool ready;
} tof_dev_t;

static tof_dev_t s_tof[2] =
{
    { TOF_FRONT_ADDR, TOF_FRONT_XSHUT_PIN, false },
    { TOF_RIGHT_ADDR, TOF_RIGHT_XSHUT_PIN, false }
};

#define TOF_CAL_POINTS_MAX   12U

typedef struct
{
    float gain;
    float offset_mm;
    uint16_t raw_mm;
    bool last_valid;
    uint16_t count;
    uint16_t known_mm[TOF_CAL_POINTS_MAX];
    uint16_t sample_raw_mm[TOF_CAL_POINTS_MAX];
} tof_cal_t;

static tof_cal_t s_cal[2];

static void tof_xshut(uint8_t sensor, GPIO_PinState state)
{
    HAL_GPIO_WritePin(TOF_XSHUT_PORT, s_tof[sensor].xshut_pin, state);
}

static bool tof_wait_boot(uint8_t address)
{
    uint8_t boot = 0U;
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < 100U)
    {
        if ((VL53L1X_BootState(address, &boot) == 0) && ((boot & 0x01U) != 0U))
        {
            return true;
        }
        HAL_Delay(2U);
    }
    return false;
}

static bool tof_configure(uint8_t address)
{
    if (VL53L1X_StopRanging(address) != 0)
    {
        return false;
    }
    if (VL53L1X_SetDistanceMode(address, 2U) != 0)          /* 长距离模式 */
    {
        return false;
    }
    if (VL53L1X_SetTimingBudgetInMs(address, TOF_TIMING_BUDGET_MS) != 0)
    {
        return false;
    }
    if (VL53L1X_SetInterMeasurementInMs(address, TOF_INTER_MEAS_MS) != 0)
    {
        return false;
    }
    (void)VL53L1X_SetOffset(address, 0);
    (void)VL53L1X_SetXtalk(address, 0);
    return true;
}

static bool tof_init_one(uint8_t sensor)
{
    uint8_t address = s_tof[sensor].address;

    tof_xshut(sensor, GPIO_PIN_RESET);
    HAL_Delay(10U);
    tof_xshut(sensor, GPIO_PIN_SET);
    HAL_Delay(5U);

    if (!tof_wait_boot(TOF_DEFAULT_ADDR))
    {
        return false;
    }

    /* 在默认地址完成初始化，再把地址改到各自专属地址。 */
    if (VL53L1X_SensorInit(TOF_DEFAULT_ADDR) != 0)
    {
        return false;
    }
    if (VL53L1X_SetI2CAddress(TOF_DEFAULT_ADDR, address) != 0)
    {
        return false;
    }
    HAL_Delay(2U);

    if (!tof_wait_boot(address))
    {
        return false;
    }
    return tof_configure(address);
}

uint8_t tof400_init(void)
{
    uint8_t result = 0U;
    uint8_t i;

    for (i = 0U; i < 2U; i++)
    {
        s_cal[i].gain = 1.0f;
        s_cal[i].offset_mm = 0.0f;
        s_cal[i].raw_mm = 0U;
        s_cal[i].last_valid = false;
        s_cal[i].count = 0U;
    }

    /* 两片都先保持复位，避免地址冲突。 */
    tof_xshut(TOF_FRONT, GPIO_PIN_RESET);
    tof_xshut(TOF_RIGHT, GPIO_PIN_RESET);
    HAL_Delay(20U);

    s_tof[TOF_FRONT].ready = tof_init_one(TOF_FRONT);
    if (!s_tof[TOF_FRONT].ready)
    {
        tof_xshut(TOF_FRONT, GPIO_PIN_RESET);
    }

    s_tof[TOF_RIGHT].ready = tof_init_one(TOF_RIGHT);
    if (!s_tof[TOF_RIGHT].ready)
    {
        tof_xshut(TOF_RIGHT, GPIO_PIN_RESET);
    }

    /* 两片都完成初始化后再错开启动，避免两束激光同时发射。 */
    if (s_tof[TOF_FRONT].ready)
    {
        if (VL53L1X_StartRanging(s_tof[TOF_FRONT].address) == 0)
        {
            result |= 0x01U;
        }
        else
        {
            s_tof[TOF_FRONT].ready = false;
        }
    }

    HAL_Delay(TOF_PHASE_OFFSET_MS);

    if (s_tof[TOF_RIGHT].ready)
    {
        if (VL53L1X_StartRanging(s_tof[TOF_RIGHT].address) == 0)
        {
            result |= 0x02U;
        }
        else
        {
            s_tof[TOF_RIGHT].ready = false;
        }
    }

    return result;
}

bool tof400_read(uint8_t sensor, tof_sample_t *sample)
{
    uint8_t data_ready = 0U;
    VL53L1X_Result_t result;

    if ((sensor > TOF_RIGHT) || (sample == NULL) || (!s_tof[sensor].ready))
    {
        return false;
    }

    if (VL53L1X_CheckForDataReady(s_tof[sensor].address, &data_ready) != 0)
    {
        return false;
    }
    if (data_ready == 0U)
    {
        return false;
    }

    if (VL53L1X_GetResult(s_tof[sensor].address, &result) != 0)
    {
        (void)VL53L1X_ClearInterrupt(s_tof[sensor].address);
        return false;
    }
    (void)VL53L1X_ClearInterrupt(s_tof[sensor].address);

    {
        float corrected = ((float)result.Distance * s_cal[sensor].gain) +
                          s_cal[sensor].offset_mm;
        int32_t corrected_i = (int32_t)(corrected + 0.5f);

        if (corrected_i < 0)
        {
            corrected_i = 0;
        }
        if (corrected_i > 65535)
        {
            corrected_i = 65535;
        }

        sample->raw_mm = result.Distance;
        sample->distance_mm = (uint16_t)corrected_i;
        s_cal[sensor].raw_mm = result.Distance;
    }
    sample->range_status = result.Status;
    sample->timestamp_ms = HAL_GetTick();
    sample->valid = (result.Status <= TOF_RANGE_STATUS_MAX) &&
                    (sample->raw_mm >= TOF_RANGE_MIN_MM) &&
                    (sample->raw_mm <= TOF_RANGE_MAX_MM) &&
                    (sample->distance_mm >= TOF_RANGE_MIN_MM) &&
                    (sample->distance_mm <= TOF_RANGE_MAX_MM);
    s_cal[sensor].last_valid = sample->valid;
    return true;
}

const char *tof400_name(uint8_t sensor)
{
    return (sensor == TOF_FRONT) ? "FRONT" : "RIGHT";
}

bool tof400_set_calibration(uint8_t sensor, float gain, float offset)
{
    if ((sensor > TOF_RIGHT) || (gain < 0.5f) || (gain > 1.5f) ||
        (offset < -300.0f) || (offset > 300.0f))
    {
        return false;
    }
    s_cal[sensor].gain = gain;
    s_cal[sensor].offset_mm = offset;
    return true;
}

bool tof400_get_calibration(uint8_t sensor, float *gain, float *offset)
{
    if ((sensor > TOF_RIGHT) || (gain == NULL) || (offset == NULL))
    {
        return false;
    }
    *gain = s_cal[sensor].gain;
    *offset = s_cal[sensor].offset_mm;
    return true;
}

bool tof400_get_last_raw(uint8_t sensor, uint16_t *raw_mm)
{
    if ((sensor > TOF_RIGHT) || (raw_mm == NULL) ||
        (s_cal[sensor].raw_mm == 0U))
    {
        return false;
    }
    *raw_mm = s_cal[sensor].raw_mm;
    return true;
}

bool tof400_calib_start(uint8_t sensor)
{
    if (sensor > TOF_RIGHT)
    {
        return false;
    }
    s_cal[sensor].count = 0U;
    s_cal[sensor].raw_mm = 0U;
    s_cal[sensor].last_valid = false;
    return true;
}

bool tof400_calib_add(uint8_t sensor, uint16_t known_mm)
{
    if ((sensor > TOF_RIGHT) || (!s_cal[sensor].last_valid) ||
        (s_cal[sensor].raw_mm == 0U) ||
        (s_cal[sensor].count >= TOF_CAL_POINTS_MAX))
    {
        return false;
    }

    s_cal[sensor].known_mm[s_cal[sensor].count] = known_mm;
    s_cal[sensor].sample_raw_mm[s_cal[sensor].count] = s_cal[sensor].raw_mm;
    s_cal[sensor].count++;
    return true;
}

uint8_t tof400_calib_count(uint8_t sensor)
{
    if (sensor > TOF_RIGHT)
    {
        return 0U;
    }
    return (uint8_t)s_cal[sensor].count;
}

bool tof400_calib_fit(uint8_t sensor, float *gain_out, float *offset_out)
{
    const tof_cal_t *cal;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_xx = 0.0f;
    float sum_xy = 0.0f;
    float denom;
    float gain;
    float offset;
    uint16_t i;

    if ((sensor > TOF_RIGHT) || (s_cal[sensor].count < 2U))
    {
        return false;
    }

    cal = &s_cal[sensor];
    for (i = 0U; i < cal->count; i++)
    {
        float x = (float)cal->sample_raw_mm[i];
        float y = (float)cal->known_mm[i];
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
    }

    denom = ((float)cal->count * sum_xx) - (sum_x * sum_x);
    if (fabsf(denom) < 1.0e-6f)
    {
        return false;
    }

    gain = (((float)cal->count * sum_xy) - (sum_x * sum_y)) / denom;
    offset = (sum_y - (gain * sum_x)) / (float)cal->count;

    if ((gain < 0.5f) || (gain > 1.5f) ||
        (offset < -300.0f) || (offset > 300.0f))
    {
        return false;
    }

    s_cal[sensor].gain = gain;
    s_cal[sensor].offset_mm = offset;
    if (gain_out != NULL)
    {
        *gain_out = gain;
    }
    if (offset_out != NULL)
    {
        *offset_out = offset;
    }
    return true;
}

void tof400_calib_reset(uint8_t sensor)
{
    if (sensor > TOF_RIGHT)
    {
        return;
    }
    s_cal[sensor].gain = 1.0f;
    s_cal[sensor].offset_mm = 0.0f;
    s_cal[sensor].count = 0U;
}
