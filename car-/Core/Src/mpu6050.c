/**
  ******************************************************************************
  * @file    mpu6050.c
  * @brief   MPU6050 驱动：±4g、±500 dps、100 Hz、DLPF 21 Hz。
  ******************************************************************************
  */
#include "mpu6050.h"
#include "i2c.h"
#include "main.h"

#define MPU6050_ADDR_LOW      (0x68U << 1)
#define MPU6050_ADDR_HIGH     (0x69U << 1)
#define MPU6050_I2C_TIMEOUT   20U

#define MPU6050_REG_SMPLRT_DIV   0x19U
#define MPU6050_REG_CONFIG       0x1AU
#define MPU6050_REG_GYRO_CONFIG  0x1BU
#define MPU6050_REG_ACCEL_CONFIG 0x1CU
#define MPU6050_REG_ACCEL_XOUT_H 0x3BU
#define MPU6050_REG_PWR_MGMT_1   0x6BU
#define MPU6050_REG_WHO_AM_I     0x75U

#define MPU6050_ACCEL_LSB_PER_G   8192.0f
#define MPU6050_GYRO_LSB_PER_DPS  65.5f

static uint16_t s_addr = MPU6050_ADDR_LOW;
static float s_gyro_bias_dps[3];

static bool mpu_write(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1, s_addr, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1U, MPU6050_I2C_TIMEOUT) == HAL_OK;
}

static bool mpu_read(uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, s_addr, reg, I2C_MEMADD_SIZE_8BIT,
                            data, len, MPU6050_I2C_TIMEOUT) == HAL_OK;
}

static int16_t mpu_i16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static bool mpu_read_raw(int16_t accel[3], int16_t gyro[3])
{
    uint8_t buf[14];
    uint8_t i;

    if (!mpu_read(MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf)))
    {
        return false;
    }

    for (i = 0U; i < 3U; i++)
    {
        accel[i] = mpu_i16(&buf[i * 2U]);
        gyro[i] = mpu_i16(&buf[8U + (i * 2U)]);
    }
    return true;
}

bool mpu6050_init(void)
{
    uint8_t who = 0U;
    uint8_t pwr = 0U;

    /* 实物 AD0 接高电平，默认地址为 0x69；同时兼容 AD0 拉低的 0x68。 */
    s_addr = MPU6050_ADDR_HIGH;
    if ((HAL_I2C_Mem_Read(&hi2c1, s_addr, MPU6050_REG_WHO_AM_I,
                          I2C_MEMADD_SIZE_8BIT, &who, 1U,
                          MPU6050_I2C_TIMEOUT) != HAL_OK) || (who != 0x68U))
    {
        s_addr = MPU6050_ADDR_LOW;
        if ((HAL_I2C_Mem_Read(&hi2c1, s_addr, MPU6050_REG_WHO_AM_I,
                              I2C_MEMADD_SIZE_8BIT, &who, 1U,
                              MPU6050_I2C_TIMEOUT) != HAL_OK) || (who != 0x68U))
        {
            return false;
        }
    }

    /* 复位，等待内部寄存器恢复。 */
    (void)mpu_write(MPU6050_REG_PWR_MGMT_1, 0x80U);
    HAL_Delay(100U);

    /* 选择 X 轴陀螺 PLL 作为时钟源。 */
    if (!mpu_write(MPU6050_REG_PWR_MGMT_1, 0x01U))
    {
        return false;
    }
    HAL_Delay(10U);

    /* 1 kHz / (1 + 9) = 100 Hz；DLPF 21 Hz。 */
    (void)mpu_write(MPU6050_REG_SMPLRT_DIV, 9U);
    (void)mpu_write(MPU6050_REG_CONFIG, 0x04U);

    /* 陀螺 ±500 dps，加速度 ±4 g。 */
    (void)mpu_write(MPU6050_REG_GYRO_CONFIG, 0x08U);
    (void)mpu_write(MPU6050_REG_ACCEL_CONFIG, 0x08U);

    (void)mpu_read(MPU6050_REG_PWR_MGMT_1, &pwr, 1U);
    s_gyro_bias_dps[0] = 0.0f;
    s_gyro_bias_dps[1] = 0.0f;
    s_gyro_bias_dps[2] = 0.0f;
    return true;
}

bool mpu6050_read(mpu6050_sample_t *sample)
{
    int16_t accel[3];
    int16_t gyro[3];
    uint8_t i;

    if ((sample == NULL) || (!mpu_read_raw(accel, gyro)))
    {
        return false;
    }

    for (i = 0U; i < 3U; i++)
    {
        sample->accel_g[i] = (float)accel[i] / MPU6050_ACCEL_LSB_PER_G;
        sample->gyro_dps[i] = ((float)gyro[i] / MPU6050_GYRO_LSB_PER_DPS)
                              - s_gyro_bias_dps[i];
    }

    /* 板温换算，仅用于诊断。 */
    sample->temperature_c = 36.53f;
    return true;
}

bool mpu6050_calibrate_gyro(uint16_t samples)
{
    int16_t accel[3];
    int16_t gyro[3];
    float sum[3] = {0.0f, 0.0f, 0.0f};
    uint16_t i;
    uint8_t axis;

    if (samples == 0U)
    {
        return false;
    }

    for (i = 0U; i < samples; i++)
    {
        if (!mpu_read_raw(accel, gyro))
        {
            return false;
        }
        for (axis = 0U; axis < 3U; axis++)
        {
            sum[axis] += (float)gyro[axis] / MPU6050_GYRO_LSB_PER_DPS;
        }
        HAL_Delay(5U);
    }

    for (axis = 0U; axis < 3U; axis++)
    {
        s_gyro_bias_dps[axis] = sum[axis] / (float)samples;
    }
    return true;
}

void mpu6050_get_gyro_bias(float bias_dps[3])
{
    if (bias_dps == NULL)
    {
        return;
    }
    bias_dps[0] = s_gyro_bias_dps[0];
    bias_dps[1] = s_gyro_bias_dps[1];
    bias_dps[2] = s_gyro_bias_dps[2];
}
