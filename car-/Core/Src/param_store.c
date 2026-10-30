/**
  ******************************************************************************
  * @file    param_store.c
  * @brief   参数/标定值 Flash 持久化。
  ******************************************************************************
  */
#include "param_store.h"
#include "chassis_config.h"
#include "chassis_vars.h"
#include "motion_control.h"
#include "nav_vars.h"
#include "tof400.h"
#include "main.h"
#include <string.h>

#define PARAM_MAGIC             0x4E415631U
#define PARAM_VERSION           1U
#define PARAM_FLASH_ADDR        0x0805F800U
#define PARAM_FLASH_PAGE_SIZE   0x800U

#pragma pack(push, 1)
typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t crc;
    uint16_t max_speed_mm_s;
    uint16_t max_omega_mrad_s;
    uint8_t nav_mode;
    uint8_t reserved;
    float tof_gain[2];
    float tof_offset[2];
    float pid_kp[CHASSIS_WHEEL_COUNT];
    float pid_ki[CHASSIS_WHEEL_COUNT];
    float pid_kd[CHASSIS_WHEEL_COUNT];
} param_blob_t;
#pragma pack(pop)

static param_blob_t s_params;

static uint16_t param_crc16(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFFU;
    uint32_t i;
    uint8_t bit;

    for (i = 0U; i < length; i++)
    {
        crc ^= (uint16_t)data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = ((crc & 1U) != 0U)
                ? (uint16_t)((crc >> 1) ^ 0xA001U)
                : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

static void param_set_defaults(param_blob_t *p)
{
    uint8_t i;

    memset(p, 0, sizeof(*p));
    p->magic = PARAM_MAGIC;
    p->version = PARAM_VERSION;
    p->max_speed_mm_s = (uint16_t)(CHASSIS_MAX_LINEAR_MPS * 1000.0f);
    p->max_omega_mrad_s = (uint16_t)(CHASSIS_MAX_ANGULAR_RADPS * 1000.0f);
    p->nav_mode = 0U;

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        p->pid_kp[i] = CHASSIS_PID_KP_DEFAULT;
        p->pid_ki[i] = CHASSIS_PID_KI_DEFAULT;
        p->pid_kd[i] = CHASSIS_PID_KD_DEFAULT;
    }

    p->tof_gain[0] = 1.0f;
    p->tof_gain[1] = 1.0f;
    p->tof_offset[0] = 0.0f;
    p->tof_offset[1] = 0.0f;
}

static void param_apply(const param_blob_t *p)
{
    uint8_t i;
    float gain;
    float offset;

    motion_set_limits((float)p->max_speed_mm_s / 1000.0f,
                      (float)p->max_omega_mrad_s / 1000.0f);
    (void)give_nav_mode(p->nav_mode);

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        (void)give_pid_gains(i, p->pid_kp[i], p->pid_ki[i], p->pid_kd[i]);
    }

    for (i = 0U; i < 2U; i++)
    {
        gain = p->tof_gain[i];
        offset = p->tof_offset[i];
        (void)tof400_set_calibration(i, gain, offset);
    }
}

static void param_capture(param_blob_t *p)
{
    motion_status_t motion;
    pid_gains_t gains;
    uint8_t i;

    p->magic = PARAM_MAGIC;
    p->version = PARAM_VERSION;
    p->nav_mode = get_nav_mode();

    motion_get_status(&motion);
    p->max_speed_mm_s = (uint16_t)(motion.max_speed_mps * 1000.0f);
    p->max_omega_mrad_s = (uint16_t)(motion.max_omega_radps * 1000.0f);

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        get_pid_gains(i, &gains);
        p->pid_kp[i] = gains.kp;
        p->pid_ki[i] = gains.ki;
        p->pid_kd[i] = gains.kd;
    }
    for (i = 0U; i < 2U; i++)
    {
        (void)tof400_get_calibration(i, &p->tof_gain[i], &p->tof_offset[i]);
    }
}

void param_store_init(void)
{
    param_set_defaults(&s_params);
    if (!param_store_load())
    {
        param_apply(&s_params);
    }
}

bool param_store_load(void)
{
    param_blob_t temp;
    uint16_t crc;

    memcpy(&temp, (const void *)PARAM_FLASH_ADDR, sizeof(temp));
    if ((temp.magic != PARAM_MAGIC) || (temp.version != PARAM_VERSION))
    {
        return false;
    }

    crc = temp.crc;
    temp.crc = 0U;
    if (crc != param_crc16((const uint8_t *)&temp, sizeof(temp)))
    {
        return false;
    }

    temp.crc = crc;
    s_params = temp;
    param_apply(&s_params);
    return true;
}

bool param_store_save(void)
{
    param_blob_t temp;
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error = 0U;
    uint16_t *src;
    uint32_t halfwords;
    uint32_t i;
    bool ok = false;

    if (get_chassis_enabled())
    {
        return false;
    }

    {
        motion_status_t motion;
        motion_get_status(&motion);
        if ((motion.state == MOTION_RUNNING) || (motion.building != 0U))
        {
            return false;
        }
    }

    param_capture(&temp);
    temp.crc = 0U;
    temp.crc = param_crc16((const uint8_t *)&temp, sizeof(temp));

    if (sizeof(temp) > PARAM_FLASH_PAGE_SIZE)
    {
        return false;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.PageAddress = PARAM_FLASH_ADDR;
    erase.NbPages = 1U;
    if (HAL_FLASHEx_Erase(&erase, &page_error) == HAL_OK)
    {
        src = (uint16_t *)&temp;
        halfwords = ((uint32_t)sizeof(temp) + 1U) / 2U;
        ok = true;
        for (i = 0U; i < halfwords; i++)
        {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                                  PARAM_FLASH_ADDR + (i * 2U),
                                  src[i]) != HAL_OK)
            {
                ok = false;
                break;
            }
        }
        if (ok)
        {
            param_blob_t verify;
            memcpy(&verify, (const void *)PARAM_FLASH_ADDR, sizeof(verify));
            if ((verify.magic != PARAM_MAGIC) || (verify.crc != temp.crc))
            {
                ok = false;
            }
        }
    }

    (void)HAL_FLASH_Lock();
    return ok;
}

bool param_store_reset_defaults(void)
{
    param_set_defaults(&s_params);
    param_apply(&s_params);
    return param_store_save();
}

bool param_store_get_value(uint8_t param_id, int32_t *value)
{
    if (value == NULL)
    {
        return false;
    }

    switch (param_id)
    {
    case PARAM_ID_MAX_SPEED:
        *value = (int32_t)s_params.max_speed_mm_s;
        break;
    case PARAM_ID_MAX_OMEGA:
        *value = (int32_t)s_params.max_omega_mrad_s;
        break;
    case PARAM_ID_NAV_MODE:
        *value = (int32_t)s_params.nav_mode;
        break;
    case PARAM_ID_TOF_FRONT_GAIN:
    case PARAM_ID_TOF_FRONT_OFFSET:
    case PARAM_ID_TOF_RIGHT_GAIN:
    case PARAM_ID_TOF_RIGHT_OFFSET:
    {
        float gain;
        float offset;
        uint8_t sensor = ((param_id == PARAM_ID_TOF_FRONT_GAIN) ||
                          (param_id == PARAM_ID_TOF_FRONT_OFFSET)) ? 0U : 1U;
        if (!tof400_get_calibration(sensor, &gain, &offset))
        {
            return false;
        }
        if ((param_id == PARAM_ID_TOF_FRONT_GAIN) ||
            (param_id == PARAM_ID_TOF_RIGHT_GAIN))
        {
            *value = (int32_t)(gain * 1000.0f);
        }
        else
        {
            *value = (int32_t)offset;
        }
        break;
    }
    default:
        return false;
    }
    return true;
}

bool param_store_set_value(uint8_t param_id, int32_t value)
{
    float gain;
    float offset;

    switch (param_id)
    {
    case PARAM_ID_MAX_SPEED:
        if ((value < 50) || (value > 800))
        {
            return false;
        }
        s_params.max_speed_mm_s = (uint16_t)value;
        motion_set_limits((float)value / 1000.0f,
                          (float)s_params.max_omega_mrad_s / 1000.0f);
        break;
    case PARAM_ID_MAX_OMEGA:
        if ((value < 100) || (value > 2500))
        {
            return false;
        }
        s_params.max_omega_mrad_s = (uint16_t)value;
        motion_set_limits((float)s_params.max_speed_mm_s / 1000.0f,
                          (float)value / 1000.0f);
        break;
    case PARAM_ID_NAV_MODE:
        if (value > 1)
        {
            return false;
        }
        s_params.nav_mode = (uint8_t)value;
        (void)give_nav_mode((uint8_t)value);
        break;
    case PARAM_ID_TOF_FRONT_GAIN:
    case PARAM_ID_TOF_RIGHT_GAIN:
    {
        uint8_t sensor = (param_id == PARAM_ID_TOF_FRONT_GAIN) ? 0U : 1U;
        gain = (float)value / 1000.0f;
        if (!tof400_get_calibration(sensor, &s_params.tof_gain[sensor], &offset))
        {
            return false;
        }
        if (!tof400_set_calibration(sensor, gain, offset))
        {
            return false;
        }
        s_params.tof_gain[sensor] = gain;
        break;
    }
    case PARAM_ID_TOF_FRONT_OFFSET:
    case PARAM_ID_TOF_RIGHT_OFFSET:
    {
        uint8_t sensor = (param_id == PARAM_ID_TOF_FRONT_OFFSET) ? 0U : 1U;
        if (!tof400_get_calibration(sensor, &gain, &s_params.tof_offset[sensor]))
        {
            return false;
        }
        offset = (float)value;
        if (!tof400_set_calibration(sensor, gain, offset))
        {
            return false;
        }
        s_params.tof_offset[sensor] = offset;
        break;
    }
    default:
        return false;
    }
    return true;
}
